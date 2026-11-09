#ifndef _GRF_OBSERVER_H
#define _GRF_OBSERVER_H

#include <stdint.h>

/* ===== GRF 地面反作用力观测器（手册第九章 T1, 2026-11-02）=====
 * 轮端自由体（本工程符号约定: F>0=伸腿, 腿把轮向下压）:
 *   N = m_w*(z̈_w + g) + F_leg,z
 *   接地静态:  N ≈ 机体分摊重 + 轮重（实测整车 11.5kg → 单轮约 56N）
 *   悬空自由落体: z̈→-g, F_leg→0 → N→0
 * F_leg 由 8009 反馈力矩经 VMC 逆关系反推（2026-11-02 静态账验证,
 * 与 Fleg_cmd 吻合 0.2% 以内, 左腿电机坐标镜像已处理）;
 * z̈_w 由 IMU 比力旋转到竖直方向后减 g。
 * 注意: 用机体竖直加速度近似轮心竖直加速度, 腿长快速变化时有误差;
 * 触地/离地门控场景腿长近似不变或自由落体, 该近似可用。
 * 全部输出经一阶 LPF, grf_dbg 供 ep 读取。 */

typedef struct
{
    float F_leg_l;     /* 左腿轴向力（反馈力矩反推, N, LPF 后） */
    float F_leg_r;     /* 右腿轴向力 */
    float Flegz_l;     /* 左腿竖直分量 F*cos(θ)（N, LPF 后） */
    float Flegz_r;
    float zdd_w;       /* 竖直加速度估计（m/s², LPF 后, 竖直向上为正） */
    float F_n_l;       /* 左轮地面反作用力（N, LPF 后） */
    float F_n_r;       /* 右轮地面反作用力 */
    float F_n_l_raw;   /* 未滤波原值（看噪声/调 LPF 用） */
    float F_n_r_raw;
    uint8_t air_state; /* T2 悬空状态机: 0=触地 1=悬空 */
} GRF_Dbg_s;

extern volatile GRF_Dbg_s grf_dbg;

extern volatile float grf_m_w;        /* 单侧轮+轮轴组件质量 kg（参数表 m_u） */
extern volatile float grf_accel_fc;   /* z̈ LPF 截止频率 Hz */
extern volatile float grf_fleg_fc;    /* 腿力 LPF 截止频率 Hz */

/* ===== T2 悬空门控（迟滞+驻留+死延时激活）===== */
extern volatile float grf_air_th_off;      /* 离地门限 N：双侧 F_n 低于此 */
extern volatile float grf_air_th_on;       /* 落地门限 N：双侧 F_n 高于此（迟滞） */
extern volatile uint16_t grf_air_hold_off; /* 离地判定驻留 tick @1kHz */
extern volatile uint16_t grf_air_hold_on;  /* 落地判定驻留 tick @1kHz */
extern volatile uint16_t grf_air_delay;    /* 死延时 ms：上电后检测静默期,期间 air 恒 0 */
uint8_t GRF_Airborne(void);                /* 1=悬空 0=触地（本拍状态机输出） */
void GRF_RestartAirDelay(void);            /* 重启死延时+清悬空锁存(恢复流程 S4 进入沿调用) */

void GRF_Init(void);
/* @param r_Tf/r_Tb 右腿前后关节反馈力矩（电机坐标, N·m）
 * @param l_Tf/l_Tb 左腿前后关节反馈力矩（电机坐标, N·m）
 * @param dt        调用周期 s */
void GRF_Update(float r_Tf, float r_Tb, float l_Tf, float l_Tb, float dt);

#endif
