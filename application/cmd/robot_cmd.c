// app
#include "robot_def.h"
#include "robot_cmd.h"
// module
#include "remote_control.h"
#include "ins_task.h"
#include "master_process.h"
#include "message_center.h"
#include "general_def.h"
#include "dji_motor.h"
#include "bmi088.h"
// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"

// 私有宏,自动将编码器转换成角度值
#define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI) // 对齐时的角度,0-360
#define PTICH_HORIZON_ANGLE (PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI) // pitch水平时电机的角度,0-360

/* cmd应用包含的模块实例指针和交互信息存储*/
#ifdef GIMBAL_BOARD // 对双板的兼容,条件编译
#include "can_comm.h"
static CANCommInstance *cmd_can_comm; // 双板通信
#endif
#ifdef ONE_BOARD
static Publisher_t *chassis_cmd_pub;   // 底盘控制消息发布者
static Subscriber_t *chassis_feed_sub; // 底盘反馈信息订阅者
#endif                                 // ONE_BOARD

static Chassis_Ctrl_Cmd_s chassis_cmd_send;      // 发送给底盘应用的信息,包括控制信息和UI绘制相关
static Chassis_Upload_Data_s chassis_fetch_data; // 从底盘应用接收的反馈信息信息,底盘功率枪口热量与底盘运动状态等

static RC_ctrl_t *rc_data;              // 遥控器数据,初始化时返回
static Vision_Recv_s *vision_recv_data; // 视觉接收数据指针,初始化时返回
static Vision_Send_s vision_send_data;  // 视觉发送数据

static Publisher_t *gimbal_cmd_pub;            // 云台控制消息发布者
static Subscriber_t *gimbal_feed_sub;          // 云台反馈信息订阅者
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send;      // 传递给云台的控制信息
static Gimbal_Upload_Data_s gimbal_fetch_data; // 从云台获取的反馈信息

static Publisher_t *shoot_cmd_pub;           // 发射控制消息发布者
static Subscriber_t *shoot_feed_sub;         // 发射反馈信息订阅者
static Shoot_Ctrl_Cmd_s shoot_cmd_send;      // 传递给发射的控制信息
static Shoot_Upload_Data_s shoot_fetch_data; // 从发射获取的反馈信息
static uint8_t keyboard_shoot_allowed;
static uint8_t keyboard_shoot_last_key_count;
volatile uint8_t keyboard_shoot_fire_active;

static Robot_Status_e robot_state; // 机器人整体工作状态

BMI088Instance *bmi088_test; // 云台IMU
BMI088_Data_t bmi088_data;
void RobotCMDInit()
{
    // BMI088_Init_Config_s bmi088_config = {
    //     .cali_mode = BMI088_CALIBRATE_ONLINE_MODE,
    //     .work_mode = BMI088_BLOCK_TRIGGER_MODE,
    //     .spi_acc_config = {
    //         .spi_handle = &hspi1,
    //         .GPIOx = GPIOA,
    //         .cs_pin = GPIO_PIN_4,
    //         .spi_work_mode = SPI_DMA_MODE,
    //     },
    //     .acc_int_config = {
    //         .GPIOx = GPIOC,
    //         .GPIO_Pin = GPIO_PIN_4,
    //         .exti_mode = GPIO_EXTI_MODE_RISING,
    //     },
    //     .spi_gyro_config = {
    //         .spi_handle = &hspi1,
    //         .GPIOx = GPIOB,
    //         .cs_pin = GPIO_PIN_0,
    //         .spi_work_mode = SPI_DMA_MODE,
    //     },
    //     .gyro_int_config = {
    //         .GPIO_Pin = GPIO_PIN_5,
    //         .GPIOx = GPIOC,
    //         .exti_mode = GPIO_EXTI_MODE_RISING,
    //     },
    //     .heat_pwm_config = {
    //         .htim = &htim10,
    //         .channel = TIM_CHANNEL_1,
    //         .period = 1,
    //     },
    //     .heat_pid_config = {
    //         .Kp = 0.5,
    //         .Ki = 0,
    //         .Kd = 0,
    //         .DeadBand = 0.1,
    //         .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
    //         .IntegralLimit = 100,
    //         .MaxOut = 100,
    //     },
    // };
    //bmi088_test = BMI088Register(&bmi088_config);
   rc_data = RemoteControlInit(&huart3);   // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
    vision_recv_data = VisionInit(&huart1); // 视觉通信串口

    gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    shoot_cmd_pub = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    shoot_feed_sub = SubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));

#ifdef ONE_BOARD // 双板兼容
    chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    CANComm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan1,
            .tx_id = 0x312,
            .rx_id = 0x311,
        },
        .recv_data_len = sizeof(Chassis_Upload_Data_s),
        .send_data_len = sizeof(Chassis_Ctrl_Cmd_s),
    };
    cmd_can_comm = CANCommInit(&comm_conf);
#endif // GIMBAL_BOARD
    gimbal_cmd_send.pitch = 0;

    robot_state = ROBOT_READY; // 启动时机器人进入工作模式,后续加入所有应用初始化完成之后再进入
}

/**
 * @brief 根据gimbal app传回的当前电机角度计算和零位的误差
 *        单圈绝对角度的范围是0~360,说明文档中有图示
 *
 */
static void CalcOffsetAngle()
{
    // 别名angle提高可读性,不然太长了不好看,虽然基本不会动这个函数
    static float angle;
    angle = gimbal_fetch_data.yaw_motor_single_round_angle; // 从云台获取的当前yaw电机单圈角度
#if YAW_ECD_GREATER_THAN_4096                               // 如果大于180度
    if (angle > YAW_ALIGN_ANGLE && angle <= 180.0f + YAW_ALIGN_ANGLE)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    else if (angle > 180.0f + YAW_ALIGN_ANGLE)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE - 360.0f;
    else
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
#else // 小于180度
    if (angle > YAW_ALIGN_ANGLE)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    else if (angle <= YAW_ALIGN_ANGLE && angle >= YAW_ALIGN_ANGLE - 180.0f)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    else
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE + 360.0f;
#endif
}

/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *        FS-i6X + iA6B (SBUS),仅实现轮腿的前后移动与机体转向。
 *
 *        语义约定(chassis侧按 chassis_mode 解读):
 *          vx  = 前后速度指令 ds_cmd,单位 m/s,前进为正
 *          vy  = 未使用,恒0
 *          wz  = 转向角速度指令,单位 rad/s,左转为正
 *          chassis_mode = CHASSIS_WHEELLEG_DRIVE(行驶) / CHASSIS_ZERO_FORCE(急停)
 *
 *        注意:本层只产生"操作手意图"(死区+缩放+低通),
 *        s_target/yaw_target 的锚定与积分在 chassis.c 目标生成层完成。
 */
static void RemoteControlSet()
{
    /* ---- 轮腿行驶参数 ---- */
#define RC_STICK_DEADBAND 20.0f  /* 摇杆死区,±660归一化计数 */
#define WHEELLEG_VX_MAX   1.0f   /* 满杆前进速度 m/s,首次实车勿超1.5 */
#define WHEELLEG_WZ_MAX   2.0f   /* 满杆转向角速度 rad/s,wz>0=左转=LQR yaw正方向 */
#define RC_CMD_LPF_ALPHA  0.15f  /* 指令一阶低通系数(200Hz,τ≈25ms) */

    static float vx_cmd_lpf = 0.0f;
    static float wz_cmd_lpf = 0.0f;

    /* 前后: 右杆竖直(CH2,两种遥控一致);
     * 转向: 左杆水平 —— DT7 是 CH2/rocker_l_,FS-i6X SBUS 是 CH4/rocker_l1(台架确认) */
#if WHEELLEG_REMOTE_MODE == WHEELLEG_REMOTE_DBUS
#define RC_STICK_TURN rocker_l_
#else
#define RC_STICK_TURN rocker_l1
#endif
    float stick_forward = (float)rc_data[TEMP].rc.rocker_r1;
    float stick_turn    = (float)rc_data[TEMP].rc.RC_STICK_TURN;

    /* 死区 + 归一化缩放 */
    float vx_cmd = 0.0f, wz_cmd = 0.0f;
    if (stick_forward > RC_STICK_DEADBAND)
        vx_cmd = (stick_forward - RC_STICK_DEADBAND) / (660.0f - RC_STICK_DEADBAND) * WHEELLEG_VX_MAX;
    else if (stick_forward < -RC_STICK_DEADBAND)
        vx_cmd = (stick_forward + RC_STICK_DEADBAND) / (660.0f - RC_STICK_DEADBAND) * WHEELLEG_VX_MAX;
    if (stick_turn > RC_STICK_DEADBAND)
        wz_cmd = -(stick_turn - RC_STICK_DEADBAND) / (660.0f - RC_STICK_DEADBAND) * WHEELLEG_WZ_MAX;
    else if (stick_turn < -RC_STICK_DEADBAND)
        wz_cmd = -(stick_turn + RC_STICK_DEADBAND) / (660.0f - RC_STICK_DEADBAND) * WHEELLEG_WZ_MAX;
    /* 转向取反的原因: 摇杆左推解析为负值(FlySky/DT7 同为右推=+),
     * 而 LQR 约定 yaw 左转为正(LQR.c 列序注释),取反后左推=wz>0=左转。 */

    /* 一阶低通,柔化推杆冲击 */
    vx_cmd_lpf += RC_CMD_LPF_ALPHA * (vx_cmd - vx_cmd_lpf);
    wz_cmd_lpf += RC_CMD_LPF_ALPHA * (wz_cmd - wz_cmd_lpf);

    chassis_cmd_send.vx           = vx_cmd_lpf;
    chassis_cmd_send.vy           = 0.0f;
    chassis_cmd_send.wz           = wz_cmd_lpf;
    chassis_cmd_send.chassis_mode = CHASSIS_WHEELLEG_DRIVE;

    /* 轮腿无云台/发射机构,保持无力与关闭,防止残留旧指令 */
    gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
    shoot_cmd_send.shoot_mode    = SHOOT_OFF;
    shoot_cmd_send.friction_mode = FRICTION_OFF;
    shoot_cmd_send.load_mode     = LOAD_STOP;
}

/**
 * @brief 输入为键鼠时模式和控制量设置
 *
 */
static void MouseKeySet()
{
    uint8_t key_count;

    float keyboard_speed;

    gimbal_cmd_send.yaw += (float)rc_data[TEMP].mouse.x / 660 * 10; // 系数待测
    gimbal_cmd_send.pitch += (float)rc_data[TEMP].mouse.y / 660 * 10;

    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_Z] % 3) // Z键设置弹速
    {
    case 0:
        shoot_cmd_send.bullet_speed = 15;
        break;
    case 1:
        shoot_cmd_send.bullet_speed = 18;
        break;
    default:
        shoot_cmd_send.bullet_speed = 30;
        break;
    }
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_E] % 4) // E键设置发射模式
    {
    case 0:
        shoot_cmd_send.load_mode = LOAD_STOP;
        break;
    case 1:
        shoot_cmd_send.load_mode = LOAD_1_BULLET;
        break;
    case 2:
        shoot_cmd_send.load_mode = LOAD_3_BULLET;
        break;
    default:
        shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
        break;
    }
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_R] % 2) // R键开关弹舱
    {
    case 0:
        shoot_cmd_send.lid_mode = LID_OPEN;
        break;
    default:
        shoot_cmd_send.lid_mode = LID_CLOSE;
        break;
    }
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_F] % 2) // F键开关摩擦轮
    {
    case 0:
        shoot_cmd_send.friction_mode = FRICTION_OFF;
        break;
    default:
        shoot_cmd_send.friction_mode = FRICTION_ON;
        break;
    }
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_C] % 4) // C键设置底盘速度
    {
    case 0:
        chassis_cmd_send.chassis_speed_buff = 40;
        break;
    case 1:
        chassis_cmd_send.chassis_speed_buff = 60;
        break;
    case 2:
        chassis_cmd_send.chassis_speed_buff = 80;
        break;
    default:
        chassis_cmd_send.chassis_speed_buff = 100;
        break;
    }

    keyboard_speed = KEYBOARD_CHASSIS_BASE_SPEED *
                     (float)chassis_cmd_send.chassis_speed_buff * 0.01f;
    if (rc_data[TEMP].key[KEY_PRESS].shift)
        keyboard_speed *= KEYBOARD_SHIFT_SPEED_SCALE;

    chassis_cmd_send.vx = rc_data[TEMP].key[KEY_PRESS].d * keyboard_speed -
                          rc_data[TEMP].key[KEY_PRESS].a * keyboard_speed;
    chassis_cmd_send.vy = rc_data[TEMP].key[KEY_PRESS].w * keyboard_speed -
                          rc_data[TEMP].key[KEY_PRESS].s * keyboard_speed;
    switch (rc_data[TEMP].key[KEY_PRESS].shift) // 待添加 按shift允许超功率 消耗缓冲能量
    {
    case 1:

        break;

    default:

        break;
    }

    key_count = rc_data[TEMP].key_count[KEY_PRESS][Key_G];
    if (key_count != keyboard_shoot_last_key_count)
        keyboard_shoot_allowed = !keyboard_shoot_allowed;
    keyboard_shoot_last_key_count = key_count;

    keyboard_shoot_fire_active = 0;
    if (!keyboard_shoot_allowed)
    {
        shoot_cmd_send.shoot_mode = SHOOT_OFF;
        shoot_cmd_send.friction_mode = FRICTION_OFF;
        shoot_cmd_send.load_mode = LOAD_STOP;
    }
    else if (rc_data[TEMP].mouse.press_l)
    {
        // The left mouse button reuses the friction-wheel start action and adds feeding.
        shoot_cmd_send.shoot_mode = SHOOT_ON;
        shoot_cmd_send.friction_mode = FRICTION_ON;
        shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
        shoot_cmd_send.shoot_rate = 8;
        keyboard_shoot_fire_active = 1;
    }
    else
    {
        // F controls pre-spin; feeding is only enabled by the left mouse button.
        shoot_cmd_send.shoot_mode = shoot_cmd_send.friction_mode == FRICTION_ON
                                         ? SHOOT_ON
                                         : SHOOT_OFF;
        shoot_cmd_send.load_mode = LOAD_STOP;
    }
}

/**
 * @brief  紧急停止: 急停开关(SwA/switch_right)下拨进入零力急停;
 *         遥控器掉线时由 RobotCMDTask 置零行驶目标保持站立。
 *         零力语义由 chassis.c 落实: 边沿触发对 6 个电机停机,机器人趴下。
 */
static void EmergencyHandler()
{
    if (switch_is_down(rc_data[TEMP].rc.switch_right) || robot_state == ROBOT_STOP)
    {
        robot_state = ROBOT_STOP;
        gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
        shoot_cmd_send.shoot_mode = SHOOT_OFF;
        shoot_cmd_send.friction_mode = FRICTION_OFF;
        shoot_cmd_send.load_mode = LOAD_STOP;
        LOGERROR("[CMD] emergency stop!");
    }
    // 急停开关拨回[上],恢复正常运行
    if (switch_is_up(rc_data[TEMP].rc.switch_right))
    {
        robot_state = ROBOT_READY;
        shoot_cmd_send.shoot_mode = SHOOT_OFF;
        LOGINFO("[CMD] reinstate, robot ready");
    }
}

/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask()
{
   // BMI088Acquire(bmi088_test,&bmi088_data) ;
    // 从其他应用获取回传数据
#ifdef ONE_BOARD
    SubGetMessage(chassis_feed_sub, (void *)&chassis_fetch_data);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    chassis_fetch_data = *(Chassis_Upload_Data_s *)CANCommGet(cmd_can_comm);
#endif // GIMBAL_BOARD
    SubGetMessage(shoot_feed_sub, &shoot_fetch_data);
    SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);

    // 根据gimbal的反馈值计算云台和底盘正方向的夹角,不需要传参,通过static私有变量完成
    CalcOffsetAngle();
#if WHEELLEG_REMOTE_MODE == WHEELLEG_REMOTE_NONE
    /* 站立调试模式: 不响应任何遥控输入,发布站立测试基线
     * (chassis侧收到 CHASSIS_STANDING_TEST 后目标全零,仅腿长保持0.25m(rec_l_rise)) */
    chassis_cmd_send.vx           = 0.0f;
    chassis_cmd_send.vy           = 0.0f;
    chassis_cmd_send.wz           = 0.0f;
    chassis_cmd_send.chassis_mode = CHASSIS_STANDING_TEST;
#else
    // 模式门控: 三档模式开关(SW2/switch_left) 下=遥控行驶, 中/上=站立锁定。
    // 键鼠控制已随 DT7 链路移除(FS-i6X 无键鼠)。
    // 注意: 无论哪种模式都必须每拍刷新 chassis_cmd_send,
    //       否则消息里会残留上一次的旧速度导致松杆后车辆继续移动。
    if (switch_is_down(rc_data[TEMP].rc.switch_left) && RemoteControlIsOnline())
        RemoteControlSet();
    else if (robot_state != ROBOT_STOP) // 站立锁定: 行驶目标清零,原地保持
    {
        chassis_cmd_send.vx           = 0.0f;
        chassis_cmd_send.vy           = 0.0f;
        chassis_cmd_send.wz           = 0.0f;
        chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
    }
    // 掉线保护: 遥控器离线时同站立锁定(目标清零保持站立);
    // 若此前已处于急停态, EmergencyHandler 会维持 CHASSIS_ZERO_FORCE 不被覆盖。
#endif

#if WHEELLEG_REMOTE_MODE != WHEELLEG_REMOTE_NONE
    EmergencyHandler(); // 处理遥控器急停(NONE模式下无遥控急停源,不编译)
#endif

    // 设置视觉发送数据,还需增加加速度和角速度数据
    // VisionSetFlag(chassis_fetch_data.enemy_color,,chassis_fetch_data.bullet_speed)

    // 推送消息,双板通信,视觉通信等
    // 其他应用所需的控制数据在remotecontrolsetmode和mousekeysetmode中完成设置
#ifdef ONE_BOARD
    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    CANCommSend(cmd_can_comm, (void *)&chassis_cmd_send);
#endif // GIMBAL_BOARD
    PubPushMessage(shoot_cmd_pub, (void *)&shoot_cmd_send);
    PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);
}
