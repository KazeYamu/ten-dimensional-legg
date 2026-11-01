#ifndef r_CHASSIS_H
#define r_CHASSIS_H
#include "ins_task.h"
#include "balance.h"

extern attitude_t *chassis_IMU_data;
extern r_leg_Resolve r_leg;
extern l_leg_Resolve l_leg;
extern r_Wheel_data r_wheel;
extern l_Wheel_data l_wheel;

extern four_control_data four_control_datas;
extern ten_balance_target ten_targets;
extern ten_balance_observation ten_balance_observe;



extern l_leg_motor_torque l_leg_motor_torques;


/* Joint reference positions in radians; adjust in debugger at calibration pose. */
extern volatile float l_front_zero, l_behind_zero;
/* +1/-1 converts raw left wheel direction to forward; confirm on hardware. */
extern volatile float l_wheel_direction;

/* 轮力矩 -> C620 电流码 的换算系数。
 * 由实测反推：819.2(码/A) / kt_out，kt_out = 0.0156(转子) * 15.8(自改减速箱) = 0.2469 N·m/A
 * 注意这是"输出端力矩"，不是电流。 */
#define WHEEL_T_PER_RAW   (3318.833f)
/* 3508 + C620 的电流码上限是 ±16384(=±20A)，换算回来就是 ±4.94 N·m。
 * dji_motor.c 里 LIMIT_MIN_MAX(set,-16384,16384) 只对 GM6020 生效，
 * M3508 是直接 (int16_t) 强转、没有限幅，所以必须在这里先钳住，
 * 否则 3318.833*8 = 26550 会溢出成负数、电流乱跳。 */
#define WHEEL_T_MAX       (4.90f)

/* 每条腿的重力前馈（2026-10-30 ep 实测修正）：
 * 站立稳态 Fleg_cmd ≈ 45~57 N/腿 → 整车 ≈ 10 kg，每腿承重 ≈ 50 N。
 * 旧值 29 N（按 ~6 kg 估）偏小，P 环把缺的 ~20 N 顶成 ~3 cm 稳态缩短
 * （实测两腿稳在 0.27 m 而非目标 0.30，偏离 LQR 设计工作点）。
 * 与腿长 PID 输出相加后作为 VMC 的虚拟腿长力 F。 */
#define LEG_GRAVITY_FF    (50.0f)

/* 腿长 P-P 级联（手册 §8.1）：F = FF + Kp*(Ldes-L) - Kv*Ldot
 * volatile 全局: 支持 ep 在线整定,无需重编译烧录。
 * Kp_pos: 位置刚度 N/m；Kv_vel: 轴向阻尼 N/(m/s)。
 * 2026-11-01 加强: 800/100 -> 1600/200,压腿长下垂差(6.5N 扰动 8mm->4mm)。
 * Ldot 一阶 LPF 系数（1kHz 循环），截止约 25 Hz */
#define LEG_VEL_LPF       (0.15f)
extern volatile float leg_kp_pos;   /* N/m, 原 800 */
extern volatile float leg_kv_vel;   /* N/(m/s), 原 100 */

/* Roll 补偿（手册 §8.2，符号按本机 IMU 实测定）：
 * ΔF = ROLL_KP*(-roll) - ROLL_KD*gyro[1]，右腿 += ΔF，左腿 -= ΔF。
 * 实测 roll=-3.9°(右倾) 时 ΔF>0 -> 右腿加力扶正。
 * 若上车后 roll 发散，把 ROLL_COMP_DIR 翻成 -1.0f。
 * 2026-11-01 加强: 100/10 -> 400/20,1.3° 时补偿力 2.3N->9N。
 * volatile 全局: 支持 ep 在线整定。 */
#define ROLL_COMP_DIR     (1.0f)
extern volatile float roll_kp;      /* N/rad, 原 100 */
extern volatile float roll_kd;      /* N/(rad/s), 原 10 */

/* 电机反馈量调试映射:在 Ozone/VarScope 全局符号里搜 chassis_dbg 即可
 * leg_*  : [0]r_front [1]r_behind [2]l_front [3]l_behind
 * wheel_*: [0]l_wheel [1]r_wheel
 * 位置单位 rad,速度单位 rad/s,符号与 ChassisTask 内的控制量一致
 *
 * lqr_x[] 是喂进 LQR 的 10 个状态（单位: m, m/s, rad, rad/s），
 * lqr_u[]     : [0]l_Tw [1]r_Tw [2]l_Tp [3]r_Tp，LQR 输出（N·m）
 * Fleg_cmd[]  : [0]左腿 [1]右腿，重力前馈+PID 后的虚拟腿长力（N）
 * joint_T_cmd : [0]r_front [1]r_behind [2]l_front [3]l_behind，实际关节力矩指令（N·m）
 * Tw_cmd[]    : [0]左轮 [1]右轮，限幅后真正下发的轮力矩（N·m）
 * 排查"往一个方向跑不停"时看这三个：
 *   phi 变负 + Tw 变正            -> 俯仰环方向对
 *   车体回正了但 Tw 不换号        -> s/ds 环没起作用
 *   Tw 在 ±4.9 之间几毫秒来回跳   -> 延迟振荡（控制频率不够） */
typedef struct
{
    float leg_pos[4];
    float leg_vel[4];
    float wheel_pos[2];
    float wheel_vel[2];

    float lqr_x[10];
    float lqr_u[4];
    float Fleg_cmd[2];
    float Ldot[2];        /* [0]左 [1]右，腿长速度估计（m/s），低通后 */
    float joint_T_cmd[4];
    float fric_ff[4];     /* [0]r_f [1]r_b [2]l_f [3]l_b，本拍实际叠加的关节摩擦前馈(N·m) */
    float wheel_ff[2];    /* [0]左轮 [1]右轮，本拍实际叠加的轮端摩擦前馈(N·m) */
    float Tw_cmd[2];
    float Roll;           /* IMU roll，度，调试 roll 补偿用 */
    float dFroll;         /* roll 补偿力差 ΔF（N），右腿为正 */
} Chassis_Dbg_s;
extern volatile Chassis_Dbg_s chassis_dbg;

/**
 * @brief 底盘应用初始化,请在开启rtos之前调用(目前会被RobotInit()调用)
 * 
 */
void ChassisInit();

/**
 * @brief 底盘应用任务,放入实时系统以一定频率运行
 * 
 */
void ChassisTask();

#endif // r_CHASSIS_H