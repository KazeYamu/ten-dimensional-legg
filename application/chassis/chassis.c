#include "chassis.h"
#include "robot_def.h"
#include "power_control.h"
#include "n630.h"
#include "super_cap.h"
#include "message_center.h"
#include "referee_task.h"
# include "dmmotor.h"
#include "general_def.h"
#include "bsp_dwt.h"
#include "referee_UI.h"
#include "arm_math.h"
#include "LQR.h"
#include "controller.h"
#include "VMC.h"
#include "grf_observer.h"
#include <math.h>


static DMMotorInstance *r_front_8009_motor,*r_behind_8009_motor;
static DMMotorInstance *l_front_8009_motor, *l_behind_8009_motor;
static DJIMotorInstance *l_wheel_motor, *r_wheel_motor;

static PIDInstance l_theta_pid;
static PIDInstance r_theta_pid;

/* cmd层指令订阅: chassis_cmd 话题,robot_cmd 200Hz发布 */
static Subscriber_t *chassis_cmd_sub;
static Chassis_Ctrl_Cmd_s chassis_cmd_recv;
#define RC_DRIVE_ACTIVE_VX 0.02f /* m/s, |vx|低于此视为松杆→定点 */

/* ===== 模型一致 yaw（2026-10-31 剪刀腿根因修复）=====
 * model_lagrange.py 的偏航状态 ψ = [Δ轮位移 + l·(θ_l−θ_r)]/(2Rl)，是"差动里程
 * 计偏航"（原地搓地旋转模型不可见）。固件此前喂 IMU 世界航向，语义分裂：
 * LQR 以为在修"腿别剪开"，实际被要求锁世界航向 → 无限剪刀循环。
 * 固件轮编码器值含 R·θ 项（s_code = x_wheel + R·θ，见 lqr_v2/design.py），展开：
 *   ψ  = (r_x − l_x)/(2Rl) + (R+l)/(2Rl)·(θ_l − θ_r)
 *   ψ̇ = (r_dx − l_dx)/(2Rl) + (R+l)/(2Rl)·(dθ_l − dθ_r)
 * 静止时不漂（无积分无陀螺），代价：站立不再锁世界航向，比赛需要时另加 IMU 外环。 */
#define TRACK_HALF_WIDTH  (0.24f)                 /* 半轮距 m,= params.py R_L */
#define WHEEL_RADIUS      (0.06f)                 /* 驱动轮半径 m,= params.py R_W */
#define YAW_COEF_WHEEL    (1.0f / (2.0f * TRACK_HALF_WIDTH))                            /* 2.083, 与腿长无关(design.py Cq 推导) */
/* YAW_COEF_LEG 已删除: 腿角项系数 (R+l)/(2R_L) 含腿长, 改为 ChassisTask 内
 * 用 rec_l_rise 现算联动, 防止改站立腿长时忘同步偏航模型(2026-11-05)。 */

/* ===== 关节库仑摩擦前馈（2026-10-30）=====
 * 实测左髋静摩擦 ≥2 N·m（右腿可随意掰动，左腿小腿需相当力气），死区让 LQR 的
 * 差动 Tp 全被右腿吸收（剪刀只走一条腿）。补偿加在各自关节、电机自身坐标：
 *   T_ff = T_c * tanh(ω / ω0)
 * tanh 平滑过零防 1kHz 抖振。T_c 为 volatile 全局，可在调试器在线调：
 * 左腿 0.5 起调（手测静摩擦大），右腿先 0（过补会变负阻尼、诱发极限环）。
 * 判据：左右腿手掰阻力对称；注意 ω=0 时补偿为 0，静止突破仍靠平衡自振。 */
volatile float l_joint_fric_t = 0.91f;   /* N·m/关节, 左腿 */
volatile float r_joint_fric_t = 0.35f;   /* N·m/关节, 右腿 */
#define FRICTION_TANH_W0  (0.15f)       /* rad/s, tanh 速度平滑尺度 */

/* ===== 轮端库仑摩擦前馈（2026-10-31）=====
 * 手感实测右轮毂电机阻尼大于左轮（自改减速箱装配差异），轮端摩擦同样吃掉
 * 平衡环的小 Tw 修正与推车时的滚动 -> 单腿摆动不对称的轮端成因。
 * 补偿按"前进为正"的轮速（r_wheel.r_dx / l_wheel.l_dx，m/s，左轮符号已翻正）
 * 加在 Tw 上、限幅之前；左轮 l_wheel_direction=-1 的坐标换算在其后，随之落位。
 * T_c volatile 在线调：右 0.10 / 左 0.05 起调（右轮阻尼大故给多），
 * 过大 = 轮速通道负阻尼 -> 平衡低频抖动，出现即减半。tanh(0)=0 静止无自发力。 */
volatile float r_wheel_fric_t = 0.15f;  /* N·m/轮, 右轮 */
volatile float l_wheel_fric_t = 0.05f;  /* N·m/轮, 左轮 */
#define WHEEL_FRICTION_TANH_V0 (0.03f)  /* m/s, tanh 速度平滑尺度(轮缘线速度) */

/* ===== 腿长刚度 + Roll 补偿（2026-11-01 加强,原为宏 800/100/100/10）=====
 * 加强依据: 右倾1.3°的静态偏移 = 腿长下垂差(6.5N/800≈8mm→1.0°) + roll补偿过弱
 * (KP=100 时 1.3° 仅出力 2.3N)。KP 翻倍 + roll KP 4 倍后残余预计 0.2~0.3°。
 * volatile 支持 ep 在线整定; 腿抖→Kp 减回 1200 或 Kv 单独加到 250;
 * 低频晃动→roll_kp 降到 250。 */
volatile float leg_kp_pos = 1600.0f;    /* N/m, 腿长位置刚度 */
volatile float leg_kv_vel = 200.0f;     /* N/(m/s), 腿长轴向阻尼 */
volatile float roll_kp    = 400.0f;     /* N/rad, roll 补偿刚度 */
volatile float roll_kd    = 20.0f;      /* N/(rad/s), roll 补偿阻尼 */

/* ===== 倒地自起参数（2026-11-01, volatile 支持 ep 在线整定）===== */
volatile uint8_t rec_enable = 0;   /* 总开关: 0=留存不启用(上电直接站立基线) 1=启用状态机 */
volatile float rec_l_min      = 0.13f;   /* 收拢位腿长 m（实车:倒地收拢物理下限约 0.124~0.13,硬顶 0.11 会一直顶力）*/
volatile float rec_l_stand    = 0.11f;   /* 起立目标腿长 m */
volatile float rec_ramp_speed = 0.0002f; /* 腿长目标变化速率 m/tick = 2cm/s @1kHz */
volatile float rec_kp_theta   = 9.7f;    /* 摆角归零 PD（与 K 矩阵 lθ 行增益同量级）*/
volatile float rec_kd_theta   = 1.23f;
volatile float rec_tp_max     = 6.0f;    /* 恢复期 Tp 限幅 N·m，防倒地姿态猛甩 */
volatile float rec_eps_theta  = 0.25f;   /* FOLD 到位阈值: |θ| rad,倒地时摆角被地面撑住,0.05 到不了 */
volatile float rec_eps_len    = 0.03f;   /* FOLD/STAND 到位阈值: |L-Ldes| m,收拢位受机械干涉 ~2cm */
/* 到位确认需要条件连续保持的时间: 2026-11-01 实车发现倒地姿态下腿长/摆角
 * 被地面和机体撑住,严格阈值永远不满足 -> 改"目标附近 + 驻留确认" */
volatile uint16_t rec_hold_ticks = 500;  /* 0.5s @1kHz */
/* 趴地恢复瞬态软化（2026-11-03）: 上电趴地 θ 误差 0.68 阶跃 -> LQR Tp 满力矩
 * 0.5s 打穿零点, 反作用力推 s 冲 0.62m / yaw 甩 1.17rad(实测 CSV)。
 * 修法 = 摆角目标从实测值以 rec_theta_rate 斜坡收向 0, LQR 全程小误差运行。
 * prone_rec_state: ep 可读, 0=判定中 1=摆角斜坡 2=腿长上升 3=完成 */
volatile uint8_t prone_rec_state = 0;
volatile float rec_theta_rate  = 6.0f;   /* rad/s, 摆角目标收拢速率 */
/* 完成判据(2026-11-03 按 230249 CSV 标定): 稳态 |θ|<=0.13/|φ|<=0.04,
 * 瞬态 θ 过冲 0.35/φ 一闪 0.10 —— 阈值取稳态 2 倍且低于瞬态峰值,
 * 驻留 0.5s 滤穿越(穿越带时长 <0.3s)。实测判成时刻 ~4.5s。 */
volatile float rec_center_band = 0.25f;  /* rad, 摆角平衡判定带 */
volatile float rec_phi_band    = 0.08f;  /* rad, 俯仰平衡判定带 */
volatile float dbg_th0_l = 0.0f;         /* ep 可读: 左腿锚定 θ0 */
volatile float dbg_th0_r = 0.0f;         /* ep 可读: 右腿锚定 θ0 */
/* 腿长上升段(态2): 判成后腿长目标从锚定实测值匀速爬向 rec_l_rise。
 * 轮不加门控(趴地位形轮全程接地, v2 的 0.27 门控假设不成立),
 * Tw 常开与已验证的平衡段行为一致。 */
volatile float rec_l_rise     = 0.25f;    /* 上升目标腿长 m = 站立/行驶腿长(唯一真源,偏航模型系数联动) */
volatile float rec_rise_speed = 0.0001f; /* m/tick = 10cm/s @1kHz */
/* FOLD 后腿长目标直接跳站立位(南航金城式,无中间斜坡), 防弹车两件套:
 * 1) 误差门限幅: |目标-实测|>rec_push_band 期间腿长 PID 分量钳到 ±rec_push_max
 *    (等效他们 stand_PID 的 20N MaxOut), 误差进带恢复 ±LEG_PID_OUT_MAX;
 * 2) 轮子门控: 实测腿长伸到 rec_l_stand-rec_wheel_margin 前轮力矩恒 0
 *    (伸到那个长度轮子必然触地), 使能沿重锚定 s/yaw 清里程漂移。 */
#define LEG_PID_OUT_MAX  (150.0f)          /* 腿长 PID 分量正常限幅 N */
volatile float rec_push_band    = 0.02f;   /* 误差门限 m */
volatile float rec_push_max     = 30.0f;   /* 大误差期间 PID 分量限幅 N */
volatile float rec_wheel_margin = 0.03f;   /* 轮使能门限 = 站立腿长 - 此值 m */
/* FOLD 专用重力前馈: 倒地时机体不压腿、腿只承自重(几N), 50N 的话
 * 收拢平衡点被抬高 50/1600≈3.1cm(≈0.14m), 腿长永远进不了 ±1cm 判据。
 * STAND 起立后腿要承重, 切回 LEG_GRAVITY_FF。 */
volatile float rec_gravity_ff = 8.0f;    /* N/腿, FOLD 阶段 */

/* ===== T3/T4 翻倒保护 + 过渡段（2026-11-05, SPR 式, volatile 供 ep 整定）=====
 * 判据与动作全部基于腿摆角 θ_body（纯编码器量, 见监控块注释）,
 * 不用欧拉角判翻倒（EKF Pitch=atan2/Roll=asinf 值域均 ±90°, 翻倒后折返）。
 * 流程: 运行中 |θ_body| 出窗 -> 脱力进态4 -> 操作手"切零力再切出"确认 ->
 *       态4 翻身(伸腿 0.40 造支点 + 追赶式恒速摆腿, SPR 同款) ->
 *       态5 收腿小板凳(L=rec_l_min, θ_body->0) -> 接现有态1 斜坡。 */
volatile uint8_t  prot_enable     = 0;      /* 0=运行中监控不跑(默认,先配数) 1=出窗脱力+进态4 */
volatile float    prot_theta_th   = 1.3f;   /* |θ_body| 出窗阈值 rad（SPR 窗口外沿 ~1.2） */
volatile uint16_t prot_hold       = 200;    /* 出窗驻留 tick @1kHz */
volatile float    prot_l_max      = 0.40f;  /* 态4 腿长目标 m: 伸到最长造支点（SPR 同款） */
volatile float    prot_l_speed    = 0.0005f;/* 态4 伸腿速率 m/tick = 50cm/s */
volatile float    prot_step       = 0.3f;   /* 追赶式摆角超前量 rad（SPR 同款, 恒速摆+抗卡滞） */
volatile float    prot_step_far   = 0.6f;   /* 双腿差>prot_diff_th 时落后腿的步长（差速纠偏） */
volatile float    prot_diff_th    = 0.8f;   /* 双腿 θ_body 差超此值触发差速 rad */
volatile float    prot_dir_sign   = 1.0f;   /* 摆腿总方向: 实测方向反了整体改 -1 */
volatile float    prot_kp_theta   = 30.0f;  /* 态4/5 θ_body PD: 追赶式下 Tp≈kp*step 恒推 */
volatile float    prot_kd_theta   = 2.0f;   /* 阻尼防摆飞 */
volatile float    prot_tp_max     = 18.0f;  /* 态4/5 Tp 限幅 N·m（要顶得动机体, 大于 rec_tp_max） */
volatile float    prot_pitch_back = -1.0f;  /* Pitch(rad) 小于此值摆腿反向（SPR :443, ±90° 内可靠） */
volatile float    prot_pitch_exit = 0.3f;   /* 态4 出口: |Pitch|<此值 才算机身翻正 */
volatile float    prot_theta_win  = 1.2f;   /* 态4 出口: 双腿 |θ_body|<此值 才算回窗口 */
volatile float    prot_band5      = 0.2f;   /* 态5 出口: 双腿 |θ_body| 进带 */
volatile uint16_t prot_timeout    = 8000;   /* 态4/5 启动后超时 tick -> 回脱力等重新切档 */
volatile uint8_t  prot_force_arm  = 0;      /* ep 置 1 跳过切档直接武装（调试用, 武装后自动清零） */

/* ===== T2 GRF 悬空门控（2026-11-03 改手册第九章降维）=====
 * 悬空时 LQR_SetAir(1): K 矩阵轮相关值清零——Tw 整行清零,
 * Tp 跳过 s|ds|yaw|dyaw 列, 腿被 LQR 主动锁在目标位(§10.3)。
 * 输出端 Tw 再硬清一道双保险, 见 ChassisTask 悬空分支。 */

/* 腿长速度估计状态（P-P 级联的阻尼项用） */
typedef struct
{
    float last_L;      /* 上次变化的腿长读数 */
    float dt_acc;      /* 距上次变化的累计时间 s */
    float raw;         /* 本拍差分原始值 */
    float filt;        /* 低通后的 Ldot */
    uint8_t started;
} LegVelState_s;
static LegVelState_s l_legvel, r_legvel;

/* Ldot 估计：DM 反馈 500Hz，1kHz 循环里差分隔拍为 0，
 * 必须按"上次变化以来的累计时间"归一，否则均值少一半。 */
static float LegVelEst(float L, LegVelState_s *st)
{
    if (!st->started)
    {
        st->last_L = L;
        st->started = 1;
        return 0.0f;
    }
    st->dt_acc += 0.001f;
    if (L != st->last_L)
    {
        st->raw = (L - st->last_L) / st->dt_acc;
        st->last_L = L;
        st->dt_acc = 0.0f;
    }
    else
    {
        st->raw = 0.0f;
    }
    st->filt += LEG_VEL_LPF * (st->raw - st->filt);
    return st->filt;
}

attitude_t *chassis_IMU_data;
r_leg_Resolve r_leg;
r_Wheel_data r_wheel;
ten_balance_observation ten_balance_observe;


four_control_data four_control_datas;
r_leg_motor_torque r_leg_motor_torques;


l_leg_Resolve l_leg;
l_Wheel_data l_wheel;



l_leg_motor_torque l_leg_motor_torques;


ten_balance_target ten_targets ;



/* Set raw joint angles measured at the mirrored right calibration pose.
 * Raw reference on right: front = rear = 0.357252 rad.
 * RAM only; fill both offsets in the debugger.
 */
volatile float l_front_zero = 0.0f, l_behind_zero = 0.0f;

volatile float l_wheel_direction = -1.0f;

volatile Chassis_Dbg_s chassis_dbg; // 仅占RAM,供调试器观测

/* 刷新电机反馈调试量,符号与ChassisTask中的控制量保持一致 */
static void ChassisDbgUpdate()
{
    const float d2r = 0.01745329252f;

    chassis_dbg.leg_pos[0] = r_front_8009_motor->measure.total_position;
    chassis_dbg.leg_pos[1] = r_behind_8009_motor->measure.total_position;
    chassis_dbg.leg_pos[2] = -l_front_8009_motor->measure.total_position;
    chassis_dbg.leg_pos[3] = -l_behind_8009_motor->measure.total_position;

    chassis_dbg.leg_vel[0] = r_front_8009_motor->measure.velocity;
    chassis_dbg.leg_vel[1] = r_behind_8009_motor->measure.velocity;
    chassis_dbg.leg_vel[2] = -l_front_8009_motor->measure.velocity;
    chassis_dbg.leg_vel[3] = -l_behind_8009_motor->measure.velocity;

    chassis_dbg.wheel_pos[0] = -l_wheel_motor->measure.total_angle * d2r;
    chassis_dbg.wheel_pos[1] = r_wheel_motor->measure.total_angle * d2r;
    chassis_dbg.wheel_vel[0] = -l_wheel_motor->measure.speed_aps * d2r;
    chassis_dbg.wheel_vel[1] = r_wheel_motor->measure.speed_aps * d2r;
}

/* 把喂进 LQR 的状态拷一份出来,方便在 Ozone 里和 T_lw 对着看 */
static void ChassisDbgLqr(const ten_balance_observation *o)
{
    chassis_dbg.lqr_x[0] = o->s;        chassis_dbg.lqr_x[1] = o->ds;
    chassis_dbg.lqr_x[2] = o->yaw;      chassis_dbg.lqr_x[3] = o->dyaw;
    chassis_dbg.lqr_x[4] = o->l_theta;  chassis_dbg.lqr_x[5] = o->l_dtheta;
    chassis_dbg.lqr_x[6] = o->r_theta;  chassis_dbg.lqr_x[7] = o->r_dtheta;
    chassis_dbg.lqr_x[8] = o->phi;      chassis_dbg.lqr_x[9] = o->dphi;
}

void ChassisInit()
{
    chassis_IMU_data = INS_Init();
    GRF_Init(); /* GRF 地面反作用力观测器（手册第九章 T1） */

    chassis_cmd_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));

    Motor_Init_Config_s r_config = {
        .can_init_config = {
            .can_handle = &hcan1,            
        },
        .controller_setting_init_config = {
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
    };

    r_config.can_init_config.tx_id = 2;
    r_config.can_init_config.rx_id = 2;
    r_front_8009_motor = DMMotorInit(&r_config);

    r_config.can_init_config.tx_id = 3;
    r_config.can_init_config.rx_id = 3;
    r_behind_8009_motor = DMMotorInit(&r_config);

    Motor_Init_Config_s l_config =
    {
        .can_init_config = {
            .can_handle = &hcan1
        },
        .controller_setting_init_config = {
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL
        },
    };

    l_config.can_init_config.tx_id = 5;
    l_config.can_init_config.rx_id = 5;
    l_front_8009_motor = DMMotorInit(&l_config);

    l_config.can_init_config.tx_id = 4;
    l_config.can_init_config.rx_id = 4;
    l_behind_8009_motor = DMMotorInit(&l_config);


    Motor_Init_Config_s chassis_motor_config = {
        .can_init_config.can_handle = &hcan2,
        .controller_param_init_config = {
            .current_PID = {
                .Kp = 5.0f,
                .Ki = 1.5f,
                .Kd = 0.0f,
                .IntegralLimit = 3000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 15000,
                .Output_LPF_RC = 0.3f,
            },
        },
        .controller_setting_init_config = {

            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,

        },

        .motor_type = M3508,

    };

    chassis_motor_config.can_init_config.tx_id = 1;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;

    l_wheel_motor = PowerControlInit(&chassis_motor_config);



    chassis_motor_config.can_init_config.tx_id = 2;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;

    r_wheel_motor = PowerControlInit(&chassis_motor_config);




    /* 腿长 P-P 级联（手册 §8.1）取代原单环 PID：
     * F = 重力前馈 + Kp_pos*(Ldes-L) - Kv_vel*Ldot
     * 正方向：F>0 = 伸腿（VMC.c 的 Tf=-gF, Tb=+gF 链路，实车已验证）。 */



    

    

}



void ChassisTask()
{
    ChassisDbgUpdate();
    /* 使能帧只在 DMMotorInit/DMMotorEnable 里发一次。原来在这里每 5ms 发两帧
     * DM_CMD_MOTOR_MODE，既占 CAN 带宽又打断 MIT 帧时序，已移除。 */
    SetPowerLimit(1000000.0f);   /* @Todo 接裁判系统后换成真实底盘功率 */

    /* ===== 接收 cmd 层指令 + 零力急停路径 =====
     * 边沿触发: Stop/Enable 只置标志,电机输出任务持续按标志发零/正常帧。 */
    SubGetMessage(chassis_cmd_sub, (void *)&chassis_cmd_recv);
    static uint8_t zero_force_last = 0;
    static uint8_t drive_last = 0; /* 行驶模式锁存,零力恢复时置0强制重新锚定 */
    static uint8_t standing_last = 0; /* 站立测试模式锁存,进入沿锚定 yaw */
    static uint8_t lock_last = 0;     /* 锁定模式锁存,进入沿冻结 s/yaw 目标 */
    uint8_t zero_force = (chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE);
    if (zero_force != zero_force_last)
    {
        if (zero_force)
        {
            DMMotorStop(r_front_8009_motor);
            DMMotorStop(r_behind_8009_motor);
            DMMotorStop(l_front_8009_motor);
            DMMotorStop(l_behind_8009_motor);
            DJIMotorStop(l_wheel_motor);
            DJIMotorStop(r_wheel_motor);
        }
        else
        {
            DMMotorEnable(r_front_8009_motor);
            DMMotorEnable(r_behind_8009_motor);
            DMMotorEnable(l_front_8009_motor);
            DMMotorEnable(l_behind_8009_motor);
            DJIMotorEnable(l_wheel_motor);
            DJIMotorEnable(r_wheel_motor);
            /* 急停期间车体可能被挪动,强制下一拍重新锚定/冻结目标 */
            drive_last = 0;
        }
        zero_force_last = zero_force;
    }
    if (zero_force)
        return; /* 停机标志已生效,本拍不再计算控制量 */

    /* 用多圈展开后的连续位置；β 的整圈歧义在 *_Alpha_beta_calculate 里归一化 */
    float r_theta_f=r_front_8009_motor->measure.total_position;
    float r_theta_b=r_behind_8009_motor->measure.total_position;
    float r_speed_f=r_front_8009_motor->measure.velocity;
    float r_speed_b=r_behind_8009_motor->measure.velocity;
    float r_wheel_angle=r_wheel_motor->measure.total_angle;
    float r_wheel_w=r_wheel_motor->measure.speed_aps;


    float l_theta_f= -l_front_8009_motor->measure.total_position;
    float l_theta_b=-l_behind_8009_motor->measure.total_position;
    float l_speed_f=-l_front_8009_motor->measure.velocity;
    float l_speed_b=-l_behind_8009_motor->measure.velocity;
    float l_wheel_angle= -l_wheel_motor->measure.total_angle;
    float l_wheel_w= -l_wheel_motor->measure.speed_aps;

    r_Alpha_beta_calculate(r_theta_f,r_theta_b,r_speed_f,r_speed_b,&r_leg);
    l_Alpha_beta_calculate(l_theta_f,l_theta_b,l_speed_f,l_speed_b,&l_leg);
    r_wheel_control(r_wheel_angle,r_wheel_w,&r_wheel);
    l_wheel_control(l_wheel_angle,l_wheel_w,&l_wheel);
    BalanceObserve(&ten_balance_observe);

    /* GRF 观测器（T1）: 反馈力矩反推腿力 + IMU 竖直加速度,输出 ep 可读 grf_dbg */
    GRF_Update(r_front_8009_motor->measure.torque, r_behind_8009_motor->measure.torque,
               l_front_8009_motor->measure.torque, l_behind_8009_motor->measure.torque,
               0.001f);

    /* ===== T3 机体系摆角 θ_body（2026-11-05, 函数级作用域）=====
     * balance.c:96 里 l_theta = θ_body − Pitch·rad, 加回来即得"腿相对机体竖直"
     * 的角 = 纯编码器运动学量, 与 IMU 姿态无关, 翻倒/欧拉角折返时依然有效
     * （数学恒等抵消, 非近似）。T3 监控、态4/5、输出闸、dbg 填充共用。 */
    float pitch_rad = chassis_IMU_data->Pitch * 0.01745329252f;
    float l_theta_body = ten_balance_observe.l_theta + pitch_rad;
    float r_theta_body = ten_balance_observe.r_theta + pitch_rad;
    static uint16_t prot_cnt = 0;
    static uint8_t  prot_armed = 0;
    static uint8_t  prot_seen_zf = 0;

    /* ===== 倒地自起 v2（2026-11-01,南航金城式:姿态纠正+直接站立+变K）=====
     * 【当前 rec_enable=0 留存不启用】台架实测: 垫 15cm 可站起, 7cm 时 DONE
     * 瞬间 LQR 在低腿长大 φ 误差下 Tp 猛甩向后倒——后续按手册第九章
     * (GRF 力观测器 + 翻倒姿态识别 + 两步恢复)整体重做,本状态机存档。
     * rec_enable=1 时恢复启用,流程:
     *   FOLD : 摆角 PD 归零 + 腿长 2cm/s 收 rec_l_min, 轮 0, LQR 不跑, 小前馈
     *   DONE : 直接交回正常控制, 腿长目标直接置 rec_l_stand(无中间斜坡段);
     *          防弹车 = 腿长环误差门限幅(rec_push_max) + 轮子在腿伸到
     *          rec_l_stand-rec_wheel_margin 前恒 0, 使能沿重锚定 s/yaw;
     *          LQR 全程在线, K 按实测腿长分带硬切(LQR.c)。
     * FOLD 摆角 PD 符号约定与 LQR 一致(Tp = Kp*(θt-θ) - Kd*dθ)。
     * 阶段切换用误差阈值+驻留确认,无固定延时。 */
    static uint8_t rec_state = 0;      /* 0=FOLD 1=DONE */
    static uint8_t rec_anchor = 0;
    static uint8_t rec_wheels_on = 0;  /* 恢复期轮使能锁存 */
    static uint8_t rec_reanchor_req = 0; /* 轮使能沿 -> 重锚定 s/yaw 请求 */
    static uint16_t rec_hold = 0;      /* 到位条件连续满足计数(驻留确认) */
    static float l_leg_ramp = 0.25f, r_leg_ramp = 0.25f;
    if (!rec_enable)
    {
        /* 总开关关闭(留存不启用): 上电直接站立基线,
         * 腿长目标恒 0.3、LQR/轮/roll 补偿全程在线,等价旧硬编码行为。
         * 注意: ep 在线把 rec_enable 写 1 不会回头跑 FOLD(rec_state 已置 1),
         * 启用状态机需改回代码重新编译烧录。 */
        rec_state = 1;
        rec_wheels_on = 1;
    }
    if (rec_state == 0)   /* FOLD: 姿态纠正 */
    {
        if (!rec_anchor)
        {
            /* 触发首拍锚定实测腿长;观测异常时夹进机械行程 */
            l_leg_ramp = ten_balance_observe.l_leg_length;
            r_leg_ramp = ten_balance_observe.r_leg_length;
            LIMIT_MIN_MAX(l_leg_ramp, 0.11f, 0.40f);
            LIMIT_MIN_MAX(r_leg_ramp, 0.11f, 0.40f);
            rec_anchor = 1;
        }
        if (l_leg_ramp > rec_l_min) l_leg_ramp -= rec_ramp_speed;
        if (r_leg_ramp > rec_l_min) r_leg_ramp -= rec_ramp_speed;
        /* 到位 = ramp 到底 + 实测进允许误差带 + 摆角接近 0,
         * 双腿条件连续保持 rec_hold_ticks 后切换(驻留确认,防瞬间满足) */
        uint8_t l_ok = (l_leg_ramp <= rec_l_min + 1e-4f) &&
                       (fabsf(ten_balance_observe.l_leg_length - rec_l_min) < rec_eps_len) &&
                       (fabsf(ten_balance_observe.l_theta) < rec_eps_theta);
        uint8_t r_ok = (r_leg_ramp <= rec_l_min + 1e-4f) &&
                       (fabsf(ten_balance_observe.r_leg_length - rec_l_min) < rec_eps_len) &&
                       (fabsf(ten_balance_observe.r_theta) < rec_eps_theta);
        rec_hold = (l_ok && r_ok) ? (uint16_t)(rec_hold + 1) : 0;
        if (rec_hold >= rec_hold_ticks)
        {
            rec_state = 1;    /* 直接进正常站立姿态,无中间变腿长过程 */
            rec_hold = 0;
        }
        /* FOLD 专属: 摆角归零 PD(符号约定与 LQR 一致) + 轮力矩 0 */
        float l_tp_rec = rec_kp_theta * (0.0f - ten_balance_observe.l_theta)
                         - rec_kd_theta * ten_balance_observe.l_dtheta;
        float r_tp_rec = rec_kp_theta * (0.0f - ten_balance_observe.r_theta)
                         - rec_kd_theta * ten_balance_observe.r_dtheta;
        LIMIT_MIN_MAX(l_tp_rec, -rec_tp_max, rec_tp_max);
        LIMIT_MIN_MAX(r_tp_rec, -rec_tp_max, rec_tp_max);
        four_control_datas.l_Tp = l_tp_rec;
        four_control_datas.r_Tp = r_tp_rec;
        four_control_datas.l_Tw = 0.0f;
        four_control_datas.r_Tw = 0.0f;
    }
    else                  /* DONE: 腿长目标恒站立位, 等腿伸到位开轮 */
    {
        l_leg_ramp = rec_l_stand;
        r_leg_ramp = rec_l_stand;
        float L_avg = 0.5f * (ten_balance_observe.l_leg_length +
                              ten_balance_observe.r_leg_length);
        if (!rec_wheels_on && L_avg >= rec_l_stand - rec_wheel_margin)
        {
            rec_wheels_on = 1;
            rec_reanchor_req = 1;   /* 轮使能沿: 清伸腿期间自由滚动的里程漂移 */
        }
    }
    ten_targets.l_leg_length_target = l_leg_ramp;
    ten_targets.r_leg_length_target = r_leg_ramp;
    chassis_dbg.rec_state = (float)rec_state;

    /* 模型一致 yaw：用里程计+腿角差重构 model_lagrange.py 的 ψ（见文件头部说明）。
     * BalanceObserve 里的 IMU yaw/dyaw 在此被整体替换；IMU 仅保留 φ/roll 用途。
     * 腿角项系数 (R+l)/(2R_L) 随站立腿长 rec_l_rise 联动, 改腿长不丢同步。 */
    float yaw_coef_leg = (WHEEL_RADIUS + rec_l_rise) / (2.0f * TRACK_HALF_WIDTH);
    ten_balance_observe.yaw = YAW_COEF_WHEEL * (r_wheel.r_x - l_wheel.l_x)
                            + yaw_coef_leg * (ten_balance_observe.l_theta - ten_balance_observe.r_theta);
    ten_balance_observe.dyaw = YAW_COEF_WHEEL * (r_wheel.r_dx - l_wheel.l_dx)
                             + yaw_coef_leg * (ten_balance_observe.l_dtheta - ten_balance_observe.r_dtheta);
    ChassisDbgLqr(&ten_balance_observe);

    /* 轮使能沿: 重锚定 s/yaw, 清掉 FOLD 后腿伸长期间轮子自由滚动累计的里程漂移 */
    if (rec_reanchor_req)
    {
        ten_targets.s_target   = ten_balance_observe.s;
        ten_targets.yaw_target = ten_balance_observe.yaw;
        rec_reanchor_req = 0;
    }

    /* ===== 遥控目标生成(cmd层意图 → LQR目标) =====
     * 行驶(CHASSIS_WHEELLEG_DRIVE): vx>阈值→控速(s_target跟随s), 松杆→定点(s_target冻结);
     *                               wz 积分为 yaw_target,松杆冻结→航向锁定。
     * 锁定(CHASSIS_NO_FOLLOW等): 进入瞬间冻结当前 s/yaw 为目标 → 原地定住。
     * 上电首拍锁定即冻结,此时 s≈0,yaw≈0,与站立调试的硬编码基线等价。
     * 倒地自起 FOLD 阶段整段跳过;DONE 恢复运行,此时 lock/drive/standing
     * 锁存均为 0,首拍走进入沿锚定。腿长目标由恢复流程恒置站立位。 */
    uint8_t air_state = GRF_Airborne(); /* T2 悬空快照,FOLD 期间也有效(轮输出端共用) */
    LQR_SetAir(air_state);              /* 悬空降维同步给 LQR(手册ch9: 轮相关通道清零) */
    if (rec_state != 0)
    {
    /* drive 在函数级作用域: 目标生成分支共用 */
    uint8_t drive = (chassis_cmd_recv.chassis_mode == CHASSIS_WHEELLEG_DRIVE);
    if (chassis_cmd_recv.chassis_mode == CHASSIS_STANDING_TEST)
    {
        /* 站立调试基线: 除 yaw 外全零(腿长0.25m=rec_l_rise已在上方设置)。
         * yaw_target 进入沿锚定当前 ψ 后冻结——里程计 ψ 会随推车/弧线漂移累计
         * 差动轮里程,若硬编码 0 会产生常值 yaw 误差 → 常值差动 Tp → 单腿被压死。
         * drive_last 置0,退出本模式后按进入沿重新锚定/冻结。 */
        if (!standing_last)
            ten_targets.yaw_target = ten_balance_observe.yaw;
        ten_targets.s_target        = 0.0f;
        ten_targets.ds_target       = 0.0f;
        ten_targets.dyaw_target     = 0.0f;
        ten_targets.l_theta_target  = 0.0f;
        ten_targets.l_dtheta_target = 0.0f;
        ten_targets.r_theta_target  = 0.0f;
        ten_targets.r_dtheta_target = 0.0f;
        ten_targets.phi_target      = 0.0f;
        ten_targets.dphi_target     = 0.0f;
        drive_last = 0;
    }
    else
    {
    if (drive && !drive_last) /* 行驶进入沿(含急停恢复后): 目标锚定当前位姿 */
    {
        ten_targets.s_target   = ten_balance_observe.s;
        ten_targets.yaw_target = ten_balance_observe.yaw;
    }
    if (drive)
    {
        if (fabsf(chassis_cmd_recv.vx) > RC_DRIVE_ACTIVE_VX)
        {
            ten_targets.ds_target = chassis_cmd_recv.vx;
            ten_targets.s_target  = ten_balance_observe.s;
        }
        else
        {
            ten_targets.ds_target = 0.0f;
        }
        ten_targets.yaw_target += chassis_cmd_recv.wz * 0.001f; /* ChassisTask 1kHz */
        ten_targets.dyaw_target = 0.0f;
    }
        else
        {
            ten_targets.ds_target = 0.0f;
            ten_targets.dyaw_target = 0.0f;
            /* 锁定进入沿: 冻结当前位姿为目标。
             * 注意不能用 !drive_last 判沿——锁定时 drive 恒 0,drive_last 恒 0,
             * 该条件每拍都真,目标会每拍跟随实测(位置环失效→持续后溜/缓转)。
             * 必须用独立的 lock_last 锁存,真·一次性进入沿。 */
            if (!lock_last)
            {
                ten_targets.s_target   = ten_balance_observe.s;
                ten_targets.yaw_target = ten_balance_observe.yaw;
            }
        }
        drive_last = drive;
    } /* end: 非站立测试模式 */
    standing_last = (chassis_cmd_recv.chassis_mode == CHASSIS_STANDING_TEST);
    lock_last = !drive && !standing_last;

    /* yaw_target 卷绕保护：目标始终与实测 ψ 取同一分支，LQR 的 x[2] 限在 ±π 内，
     * 防止 wz 长时间积分与分支卷绕差 2π 时 Tp 猛跳。 */
    float yaw_ref_err = ten_targets.yaw_target - ten_balance_observe.yaw;
    yaw_ref_err -= 2.0f * PI * roundf(yaw_ref_err / (2.0f * PI));
    ten_targets.yaw_target = ten_balance_observe.yaw + yaw_ref_err;

    /* ===== 趴地恢复瞬态软化（2026-11-03, 覆盖上面所有目标生成）=====
     * 进入: 首个有效观测帧(L>0.05, 上电前几拍读数为 0)双腿均腿长 <0.13
     *       -> 锚定 θ0 与 s/yaw 目标; 站立上电(L>=0.13)永久旁路。
     * 恢复: 摆角目标 = sign(θ0)·max(|θ0|-rate·t, 0), dθ 目标 0,
     *       LQR 见到的误差从 0 开始线性增大, Tp 恒小 -> s/yaw 不甩。
     * 完成: 双腿 |θ|<rec_center_band 驻留 rec_hold_ticks -> 退出(目标归常规)。
     * 腿长目标仍由 rec_l_stand 决定(趴地测试=0.11), 伸腿另行阶梯/斜坡。 */
    static uint16_t prone_hold = 0;
    static uint16_t prone_valid = 0;
    static float prone_t = 0.0f;
    /* T3 机体系摆角 θ_body / prot_armed 等已在本函数开头(GRF_Update 后)计算,
     * 见该处注释; 此处不再重复赋值。 */
    if (prone_rec_state == 0)
    {
        float ll = ten_balance_observe.l_leg_length;
        float rl = ten_balance_observe.r_leg_length;
        /* 双腿观测都有效且连续 50ms 才判定: 单腿先就绪时平均腿长也能过 0.05,
         * 会把另一腿的垃圾值/瞬态半途锚成 θ0(2026-11-03 实测 dbg_th0_l=-0.56) */
        if (ll > 0.05f && rl > 0.05f)
        {
            if (++prone_valid >= 50)
            {
                /* 趴地(均腿长<0.13)进摆角斜坡; 站立上电直接进上升段
                 * (锚定实测≈0.30, 斜坡空转, 且修复站立上电目标曾是 0.11 的问题) */
                prone_rec_state = (0.5f * (ll + rl) < 0.13f) ? 1 : 2;
                if (prone_rec_state == 1)
                {
                    dbg_th0_l = ten_balance_observe.l_theta;
                    dbg_th0_r = ten_balance_observe.r_theta;
                    LIMIT_MIN_MAX(dbg_th0_l, -1.4f, 1.4f);
                    LIMIT_MIN_MAX(dbg_th0_r, -1.4f, 1.4f);
                    ten_targets.s_target   = ten_balance_observe.s;
                    ten_targets.yaw_target = ten_balance_observe.yaw;
                }
            }
        }
        else
        {
            prone_valid = 0;
        }
    }
    /* ===== 态4: 翻倒过渡/翻身（2026-11-05, SPR chassis_task.c:431 同款）=====
     * 进态即脱力（输出闸清零），等操作手"切零力再切出"的确认沿才武装
     * （检测→脱力无害，动作需人确认——11-03 教训的隔离层; prot_force_arm 供 ep 跳过）。
     * 武装后: 腿长匀速伸到 prot_l_max 造支点 + θ_body 追赶式目标恒速摆
     * （目标=实测∓step 每拍重设: 腿能动则恒速摆, 被压住则目标跟着实测走、永不失配）。
     * 方向按 Pitch 符号（prot_pitch_back 反向, 仅 ±90° 可靠区间）; 双腿差大时差速纠偏。
     * 到位（机身翻正+双腿回窗）驻留 -> 态5; 超时回脱力等重新切档。 */
    static uint8_t  tilt4_anchor = 0;
    static uint16_t tilt4_t = 0;
    static uint16_t tilt4_hold = 0;
    static float tilt4_l = 0.25f, tilt4_r = 0.25f;
    if (prone_rec_state == 4)
    {
        /* 启动闸门: 先见过零力档再切出 = 操作手确认 */
        if (chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE)
            prot_seen_zf = 1;
        else if (prot_seen_zf || prot_force_arm)
        {
            prot_armed = 1;
            prot_seen_zf = 0;
            prot_force_arm = 0;
        }
        if (prot_armed)
        {
            if (!tilt4_anchor)
            {
                tilt4_l = ten_balance_observe.l_leg_length;
                tilt4_r = ten_balance_observe.r_leg_length;
                LIMIT_MIN_MAX(tilt4_l, 0.09f, 0.40f);
                LIMIT_MIN_MAX(tilt4_r, 0.09f, 0.40f);
                tilt4_anchor = 1;
                tilt4_t = 0;
                tilt4_hold = 0;
            }
            tilt4_t++;
            if (tilt4_l < prot_l_max) tilt4_l += prot_l_speed;
            if (tilt4_r < prot_l_max) tilt4_r += prot_l_speed;
            ten_targets.l_leg_length_target = tilt4_l;
            ten_targets.r_leg_length_target = tilt4_r;
            /* 追赶式摆角目标（世界系 θ 目标仅占位, LQR 输出随后被覆盖） */
            float dir = prot_dir_sign * ((pitch_rad < prot_pitch_back) ? 1.0f : -1.0f);
            float step_l = prot_step, step_r = prot_step;
            if (fabsf(l_theta_body - r_theta_body) > prot_diff_th)
            {   /* 差速: 摆角落后的腿步长加倍（SPR 固定右腿加倍, 这里按实测动态选） */
                if (fabsf(l_theta_body) > fabsf(r_theta_body)) step_r = prot_step_far;
                else                                           step_l = prot_step_far;
            }
            ten_targets.l_theta_target = l_theta_body + dir * step_l;
            ten_targets.r_theta_target = r_theta_body + dir * step_r;
            if (fabsf(pitch_rad) < prot_pitch_exit &&
                fabsf(l_theta_body) < prot_theta_win &&
                fabsf(r_theta_body) < prot_theta_win)
            {
                if (++tilt4_hold >= rec_hold_ticks)
                {
                    prone_rec_state = 5;
                    tilt4_anchor = 0;
                }
            }
            else
            {
                tilt4_hold = 0;
            }
            if (tilt4_t > prot_timeout)
            {   /* 超时: 回脱力等重新切档, 防卡死顶死。
                 * seen_zf 必须一并清零, 否则残留的"已见零力"会让操作手
                 * 不做任何确认就自动再武装（闸门失效）。 */
                prot_armed = 0;
                prot_seen_zf = 0;
                tilt4_anchor = 0;
            }
        }
    }

    /* ===== 态5: 收腿小板凳（2026-11-05, SPR FALLING_TO_STANDING:481 同款位形）=====
     * L -> rec_l_min + θ_body -> 0, 纯关节 PD, Tw=0, 小重力前馈。
     * 到位（双腿摆角进带 + 腿长到位）驻留 -> 接态1（重锚定 θ0 斜坡, 基本空转穿过）
     * -> 态2 爬升 -> 态3, 后段零改动复用。 */
    static uint8_t  tilt5_anchor = 0;
    static uint16_t tilt5_t = 0;
    static uint16_t tilt5_hold = 0;
    if (prone_rec_state == 5)
    {
        if (!tilt5_anchor)
        {
            tilt5_t = 0;
            tilt5_hold = 0;
            tilt5_anchor = 1;
        }
        tilt5_t++;
        ten_targets.l_leg_length_target = rec_l_min;
        ten_targets.r_leg_length_target = rec_l_min;
        ten_targets.l_theta_target = 0.0f;   /* 占位, LQR 输出随后被覆盖 */
        ten_targets.r_theta_target = 0.0f;
        if (fabsf(l_theta_body) < prot_band5 &&
            fabsf(r_theta_body) < prot_band5 &&
            fabsf(ten_balance_observe.l_leg_length - rec_l_min) < rec_eps_len &&
            fabsf(ten_balance_observe.r_leg_length - rec_l_min) < rec_eps_len)
        {
            if (++tilt5_hold >= rec_hold_ticks)
            {
                /* 接态1: 重锚定世界系 θ0 + s/yaw（同态0 趴地进入沿） */
                dbg_th0_l = ten_balance_observe.l_theta;
                dbg_th0_r = ten_balance_observe.r_theta;
                LIMIT_MIN_MAX(dbg_th0_l, -1.4f, 1.4f);
                LIMIT_MIN_MAX(dbg_th0_r, -1.4f, 1.4f);
                ten_targets.s_target   = ten_balance_observe.s;
                ten_targets.yaw_target = ten_balance_observe.yaw;
                prone_t = 0.0f;
                prone_hold = 0;
                prone_rec_state = 1;
                tilt5_anchor = 0;
            }
        }
        else
        {
            tilt5_hold = 0;
        }
        if (tilt5_t > prot_timeout)
        {   /* 超时: 回态4 脱力等重新切档（seen_zf 一并清零, 理由同态4 超时） */
            prot_armed = 0;
            prot_seen_zf = 0;
            tilt5_anchor = 0;
            prone_rec_state = 4;
        }
    }

    if (prone_rec_state == 1)
    {
        prone_t += 0.001f;   /* ChassisTask 1kHz */
        float step = rec_theta_rate * prone_t;
        float ml = fabsf(dbg_th0_l) - step; if (ml < 0.0f) ml = 0.0f;
        float mr = fabsf(dbg_th0_r) - step; if (mr < 0.0f) mr = 0.0f;
        ten_targets.l_theta_target  = (dbg_th0_l >= 0.0f) ? ml : -ml;
        ten_targets.r_theta_target  = (dbg_th0_r >= 0.0f) ? mr : -mr;
        ten_targets.l_dtheta_target = 0.0f;
        ten_targets.r_dtheta_target = 0.0f;
        if (fabsf(ten_balance_observe.l_theta) < rec_center_band &&
            fabsf(ten_balance_observe.r_theta) < rec_center_band &&
            fabsf(ten_balance_observe.phi)     < rec_phi_band)
        {
            if (++prone_hold >= rec_hold_ticks)
                prone_rec_state = 2;
        }
        else
        {
            prone_hold = 0;
        }
    }

    /* ===== 态2: 腿长上升段(2026-11-03) =====
     * 进入沿锚定实测腿长, 按 rec_rise_speed 匀速爬向 rec_l_rise;
     * L_avg 进 rec_l_rise-rec_eps_len 驻留 rec_hold_ticks -> 态3。
     * 轮不加门控: 趴地位形轮全程接地(实测 0.11 就在滑), v2 的"0.27 才触地"
     * 假设不成立; Tw 常开 = 已验证平衡段的配置。
     * 态3: 腿长目标恒 rec_l_rise。站立上电也走 2->3(锚定 0.30, 斜坡空转)。 */
    static uint8_t  rise_anchor = 0;
    static uint16_t rise_hold = 0;
    static float rise_l = 0.25f, rise_r = 0.25f;
    if (prone_rec_state == 2)
    {
        if (!rise_anchor)
        {
            rise_l = ten_balance_observe.l_leg_length;
            rise_r = ten_balance_observe.r_leg_length;
            LIMIT_MIN_MAX(rise_l, 0.09f, 0.40f);
            LIMIT_MIN_MAX(rise_r, 0.09f, 0.40f);
            rise_anchor = 1;
            rise_hold = 0;
        }
        if (rise_l < rec_l_rise) rise_l += rec_rise_speed;
        if (rise_r < rec_l_rise) rise_r += rec_rise_speed;
        ten_targets.l_leg_length_target = rise_l;
        ten_targets.r_leg_length_target = rise_r;
        float L_avg = 0.5f * (ten_balance_observe.l_leg_length +
                              ten_balance_observe.r_leg_length);
        if (L_avg >= rec_l_rise - rec_eps_len)
        {
            if (++rise_hold >= rec_hold_ticks)
                prone_rec_state = 3;
        }
        else
        {
            rise_hold = 0;
        }
    }
    else if (prone_rec_state == 3)
    {
        ten_targets.l_leg_length_target = rec_l_rise;
        ten_targets.r_leg_length_target = rec_l_rise;
    }

    /* ===== T3: 运行中翻倒监控（2026-11-05, SPR chassis_task.c:362 同款判据换腿摆角版）=====
     * 站立/行驶/恢复摆角期（态1/2/3）双腿任一 |θ_body| 出窗驻留 -> 脱力进态4。
     * θ_body 是纯编码器量, 不受欧拉角 ±90° 折返影响; 但腿悬空不动时不出窗
     * （云台配重下侧倒难出现, 先接受）。prot_enable=0 时整块旁路（行为同旧版）。
     * 触发时清 rise_anchor: 翻倒恢复链会再进态2, 必须重新锚定腿长起点防阶跃弹车。 */
    if (prot_enable &&
        (prone_rec_state == 1 || prone_rec_state == 2 || prone_rec_state == 3) &&
        ten_balance_observe.l_leg_length > 0.05f &&
        ten_balance_observe.r_leg_length > 0.05f)
    {
        if (fabsf(l_theta_body) > prot_theta_th || fabsf(r_theta_body) > prot_theta_th)
        {
            if (++prot_cnt >= prot_hold)
            {
                prone_rec_state = 4;
                prot_armed = 0;
                prot_seen_zf = 0;
                prot_cnt = 0;
                rise_anchor = 0;
            }
        }
        else
        {
            prot_cnt = 0;
        }
    }
    else
    {
        prot_cnt = 0;
    }

    LQR_calc(&ten_balance_observe,&ten_targets,&four_control_datas);
    chassis_dbg.lqr_u[0] = four_control_datas.l_Tw;
    chassis_dbg.lqr_u[1] = four_control_datas.r_Tw;
    chassis_dbg.lqr_u[2] = four_control_datas.l_Tp;
    chassis_dbg.lqr_u[3] = four_control_datas.r_Tp;

    /* ===== T4: 态4/5 输出覆盖（2026-11-05）=====
     * 翻倒过渡期不经 LQR（手册第九章: 自救期强制不经 LQR/VMC 平衡通道）:
     * Tp = θ_body PD（追赶式/归零目标）, Tw 硬清零, 腿长力仍走下方腿长环。
     * LQR_calc 无内部状态, 覆盖安全。θ_body 目标: 态4=追赶式, 态5=0（归位）。 */
    if ((prone_rec_state == 4 && prot_armed) || prone_rec_state == 5)
    {
        float l_tb = (prone_rec_state == 4) ? ten_targets.l_theta_target : 0.0f;
        float r_tb = (prone_rec_state == 4) ? ten_targets.r_theta_target : 0.0f;
        float lt4 = prot_kp_theta * (l_tb - l_theta_body)
                    - prot_kd_theta * ten_balance_observe.l_dtheta;
        float rt4 = prot_kp_theta * (r_tb - r_theta_body)
                    - prot_kd_theta * ten_balance_observe.r_dtheta;
        LIMIT_MIN_MAX(lt4, -prot_tp_max, prot_tp_max);
        LIMIT_MIN_MAX(rt4, -prot_tp_max, prot_tp_max);
        four_control_datas.l_Tp = lt4;
        four_control_datas.r_Tp = rt4;
        four_control_datas.l_Tw = 0.0f;
        four_control_datas.r_Tw = 0.0f;
    }

    /* ===== T2: GRF 悬空门控（2026-11-03 手册第九章降维）=====
     * 悬空时 LQR_SetAir 已把 K 矩阵轮相关值清零: Tw=0(止疯转),
     * Tp 保留 φ/φ̇+腿摆角通道 → 腿被 LQR 主动锁在目标位, 不再阻尼下垂。
     * 此处输出端 Tw 硬清一道双保险(也消 Tw 反作用对力矩反推的污染)。
     * 腿锁住后 F_leg 卸到 -m_w·g, F_n 干净归 0;
     * 落地沿(F_n 双侧>th_on 驻留)恢复 LQR 并重锚定 s/yaw 清空转里程漂移。 */
    static uint8_t air_reanchor = 0;
    if (air_state)
    {
        four_control_datas.l_Tw = 0.0f;
        four_control_datas.r_Tw = 0.0f;
        air_reanchor = 1;
    }
    else if (air_reanchor)
    {   /* 落地沿: 悬空期间轮子自由空转的里程漂移一次性清掉 */
        ten_targets.s_target   = ten_balance_observe.s;
        ten_targets.yaw_target = ten_balance_observe.yaw;
        air_reanchor = 0;
    }
    } /* end: rec_state!=0 (仅 FOLD 跳过目标生成+LQR;DONE 已恢复) */
    /* 虚拟腿长力 = 重力前馈 + 腿长 PID；VMC 内部再按修正后的 J^T 映射。 */
    float l_dot = LegVelEst(ten_balance_observe.l_leg_length, &l_legvel);
    float r_dot = LegVelEst(ten_balance_observe.r_leg_length, &r_legvel);
    float l_pid_out = leg_kp_pos * (ten_targets.l_leg_length_target - ten_balance_observe.l_leg_length)
                      - leg_kv_vel * l_dot;
    float r_pid_out = leg_kp_pos * (ten_targets.r_leg_length_target - ten_balance_observe.r_leg_length)
                      - leg_kv_vel * r_dot;
    /* 腿长误差门限幅(南航金城 stand_PID MaxOut 思路,防 FOLD 后跳目标弹车):
     * |目标-实测| 超 rec_push_band 期间 PID 分量钳 ±rec_push_max,
     * 误差进带(正常平衡/行驶,误差毫米级)恢复全量 ±LEG_PID_OUT_MAX。 */
    float l_pid_max = (fabsf(ten_targets.l_leg_length_target - ten_balance_observe.l_leg_length) > rec_push_band)
                      ? rec_push_max : LEG_PID_OUT_MAX;
    float r_pid_max = (fabsf(ten_targets.r_leg_length_target - ten_balance_observe.r_leg_length) > rec_push_band)
                      ? rec_push_max : LEG_PID_OUT_MAX;
    LIMIT_MIN_MAX(l_pid_out, -l_pid_max, l_pid_max);
    LIMIT_MIN_MAX(r_pid_out, -r_pid_max, r_pid_max);
    /* Roll 补偿（手册 §8.2）：右倾(roll<0)时右腿加力、左腿减力。
     * Roll 单位度 -> rad；Gyro[1] 为 roll 角速度(rad/s)。
     * FOLD 倒地阶段 roll 无意义,补偿关闭;DONE 恢复。 */
    float roll_rad = chassis_IMU_data->Roll * 0.01745329252f;
    float dF_roll = (rec_state == 0 || prone_rec_state == 4 || prone_rec_state == 5) ? 0.0f
                               : ROLL_COMP_DIR * (roll_kp * (-roll_rad) - roll_kd * chassis_IMU_data->Gyro[1]);
    /* 重力前馈分阶段: FOLD 用小前馈(腿不承重,见 rec_gravity_ff 注释),
     * DONE 起立要顶起机体 + 正常控制用 50N。 */
    float grav_ff = (rec_state == 0 || prone_rec_state == 4 || prone_rec_state == 5)
                    ? rec_gravity_ff : LEG_GRAVITY_FF;
    four_control_datas.l_Tleg = grav_ff + l_pid_out - dF_roll;
    four_control_datas.r_Tleg = grav_ff + r_pid_out + dF_roll;
    chassis_dbg.Fleg_cmd[0] = four_control_datas.l_Tleg;
    chassis_dbg.Fleg_cmd[1] = four_control_datas.r_Tleg;
    chassis_dbg.Ldot[0] = l_dot;
    chassis_dbg.Ldot[1] = r_dot;
    chassis_dbg.Roll = chassis_IMU_data->Roll;
    chassis_dbg.dFroll = dF_roll;
    chassis_dbg.theta_body[0] = l_theta_body;
    chassis_dbg.theta_body[1] = r_theta_body;
    chassis_dbg.prot = (float)(prot_enable | (prot_armed << 1));

    /* 输出闸: ①观测无效期(上电头几拍读数全 0, 腿长环却会以 30N/腿推腿,
     * 折叠位形下该力把腿/轮推进入瞬态, 恢复锚定就锚到运动半途
     * (2026-11-03 实测: dbg_th0_l=-0.56, 左轮 -408rad/s, s 冲 -0.69),
     * 判定完成(prone_rec_state 离开 0)前电机不出力, 腿由机械摩擦保持;
     * ②态4 未武装期(翻倒检测触发后, 等操作手切档确认, 全零脱力无害等待)。 */
    if (prone_rec_state == 0 || (prone_rec_state == 4 && !prot_armed))
    {
        four_control_datas.l_Tp = 0.0f;
        four_control_datas.r_Tp = 0.0f;
        four_control_datas.l_Tw = 0.0f;
        four_control_datas.r_Tw = 0.0f;
        four_control_datas.l_Tleg = 0.0f;
        four_control_datas.r_Tleg = 0.0f;
    }

    VMC_Calc(&four_control_datas, &r_leg_motor_torques, &r_leg);
    l_VMC_Calc(&four_control_datas, &l_leg_motor_torques, &l_leg);

    /* 关节摩擦前馈：VMC 输出的 Tf/Tb 已是电机自身坐标，与 measure.velocity 同系，
     * 直接同号叠加即可（左右腿各自的坐标约定在 VMC 内部已处理完）。 */
    float ff_rf = r_joint_fric_t * tanhf(r_front_8009_motor->measure.velocity  / FRICTION_TANH_W0);
    float ff_rb = r_joint_fric_t * tanhf(r_behind_8009_motor->measure.velocity / FRICTION_TANH_W0);
    float ff_lf = l_joint_fric_t * tanhf(l_front_8009_motor->measure.velocity  / FRICTION_TANH_W0);
    float ff_lb = l_joint_fric_t * tanhf(l_behind_8009_motor->measure.velocity / FRICTION_TANH_W0);
    r_leg_motor_torques.r_Tf += ff_rf;
    r_leg_motor_torques.r_Tb += ff_rb;
    l_leg_motor_torques.l_Tf += ff_lf;
    l_leg_motor_torques.l_Tb += ff_lb;

    chassis_dbg.joint_T_cmd[0] = r_leg_motor_torques.r_Tf;
    chassis_dbg.joint_T_cmd[1] = r_leg_motor_torques.r_Tb;
    chassis_dbg.joint_T_cmd[2] = l_leg_motor_torques.l_Tf;
    chassis_dbg.joint_T_cmd[3] = l_leg_motor_torques.l_Tb;
    chassis_dbg.fric_ff[0] = ff_rf;
    chassis_dbg.fric_ff[1] = ff_rb;
    chassis_dbg.fric_ff[2] = ff_lf;
    chassis_dbg.fric_ff[3] = ff_lb;


    // DMMotorStop(r_front_8009_motor);
    // DMMotorStop(r_behind_8009_motor);
    // DMMotorStop(l_front_8009_motor);
    // DMMotorStop(l_behind_8009_motor);
    // DJIMotorStop(l_wheel_motor);
    // DJIMotorStop(r_wheel_motor);

    DMMotorSetCtrlMode(r_front_8009_motor,  DM_CTRL_TORQUE);
    DMMotorSetCtrlMode(r_behind_8009_motor, DM_CTRL_TORQUE);
    DMMotorSetCtrlMode(l_front_8009_motor,  DM_CTRL_TORQUE);
    DMMotorSetCtrlMode(l_behind_8009_motor, DM_CTRL_TORQUE);

    /* l_VMC_Calc 已换回原电机坐标，左腿下发时不能再加负号。 */
    DMMotorSetRef(r_front_8009_motor,
        isfinite(r_leg_motor_torques.r_Tf) ? r_leg_motor_torques.r_Tf : 0.0f);
    DMMotorSetRef(r_behind_8009_motor,
        isfinite(r_leg_motor_torques.r_Tb) ? r_leg_motor_torques.r_Tb : 0.0f);
    DMMotorSetRef(l_front_8009_motor,
        isfinite(l_leg_motor_torques.l_Tf) ? l_leg_motor_torques.l_Tf : 0.0f);
    DMMotorSetRef(l_behind_8009_motor,
        isfinite(l_leg_motor_torques.l_Tb) ? l_leg_motor_torques.l_Tb : 0.0f);

    /* 轮力矩下发：先按 3508 的真实上限钳位，再换算成电流码。
     * 限幅必须放在换算之前 —— 见 chassis.h 里 WHEEL_T_MAX 的注释。 */
    /* 轮端摩擦前馈：r_dx/l_dx 已是前进为正的轮缘线速度(m/s)，直接取符号；
     * 左轮 l_wheel_direction=-1 的换算在 DMMotorSetRef 处，FF 加在换算前随之落位。
     * 仅 FOLD/EXTEND(轮未触地)阶段轮力矩恒 0;BALANCE 起轮控随 LQR 恢复。 */
    float r_Tw, l_Tw;
    if (rec_state == 0 || !rec_wheels_on || air_state ||
        prone_rec_state == 4 || prone_rec_state == 5)
    {
        /* FOLD 阶段, 或 DONE 后腿还没伸到轮使能门限(此时轮必然悬空),
         * 或 GRF 判悬空(T2): 轮力矩清零, 摩擦前馈一并跳过 */
        r_Tw = 0.0f;
        l_Tw = 0.0f;
    }
    else
    {
        float ff_rw = r_wheel_fric_t * tanhf(r_wheel.r_dx / WHEEL_FRICTION_TANH_V0);
        float ff_lw = l_wheel_fric_t * tanhf(l_wheel.l_dx / WHEEL_FRICTION_TANH_V0);
        r_Tw = four_control_datas.r_Tw + ff_rw;
        l_Tw = four_control_datas.l_Tw + ff_lw;
        LIMIT_MIN_MAX(r_Tw, -WHEEL_T_MAX, WHEEL_T_MAX);
        LIMIT_MIN_MAX(l_Tw, -WHEEL_T_MAX, WHEEL_T_MAX);
        chassis_dbg.wheel_ff[0] = ff_lw;
        chassis_dbg.wheel_ff[1] = ff_rw;
    }
    chassis_dbg.Tw_cmd[0] = l_Tw;
    chassis_dbg.Tw_cmd[1] = r_Tw;

    DJIMotorSetRef(r_wheel_motor, WHEEL_T_PER_RAW * r_Tw);
    DJIMotorSetRef(l_wheel_motor, l_wheel_direction * WHEEL_T_PER_RAW * l_Tw);

    



}
