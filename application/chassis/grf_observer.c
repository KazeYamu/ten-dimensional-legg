#include "grf_observer.h"
#include "chassis.h"
#include "balance.h"
#include <math.h>

/* 腿杆参数与 VMC.c 同源（balance.h）,g(β) 为 VMC.c 同一公式的副本:
 * 改腿杆尺寸时两处要一起改。 */
#define GRF_LEG_A  (0.21f)
#define GRF_LEG_B  (0.25f)
#define GRF_G      (9.81f)

volatile GRF_Dbg_s grf_dbg;
volatile float grf_m_w      = 0.5f;   /* 单侧轮+轮轴组件 kg,参数表 m_u,ep 可调 */
volatile float grf_accel_fc = 25.0f;  /* z̈ 低通截止 Hz,悬空判定要快,压噪靠腿力通道 */
volatile float grf_fleg_fc  = 60.0f;  /* 腿力低通截止 Hz,力矩噪声 ~0.5N,可放宽 */

/* T2 悬空门控参数：站立 F_n≈55~65N,离地沿 0.1s 内掉到 40 以下(2026-11-02 实测),
 * 门限 20/35 迟留裕量充足;颠簸误触发靠双侧同时低+驻留时间兜底。 */
volatile float grf_air_th_off = 20.0f;
volatile float grf_air_th_on  = 35.0f;
volatile uint16_t grf_air_hold_off = 50;  /* 50ms */
volatile uint16_t grf_air_hold_on  = 20;  /* 20ms */
static uint8_t grf_air = 0;
static uint16_t grf_air_cnt = 0;

uint8_t GRF_Airborne(void) { return grf_air; }

static void grf_air_update(float n_l, float n_r)
{
    if (!grf_air)
    {
        if (n_l < grf_air_th_off && n_r < grf_air_th_off)
        {
            if (++grf_air_cnt >= grf_air_hold_off) { grf_air = 1; grf_air_cnt = 0; }
        }
        else grf_air_cnt = 0;
    }
    else
    {
        if (n_l > grf_air_th_on && n_r > grf_air_th_on)
        {
            if (++grf_air_cnt >= grf_air_hold_on) { grf_air = 0; grf_air_cnt = 0; }
        }
        else grf_air_cnt = 0;
    }
}

typedef struct { float y; uint8_t started; } GRF_Lpf_s;
static GRF_Lpf_s lpf_fleg_l, lpf_fleg_r, lpf_zdd, lpf_fn_l, lpf_fn_r;

static float grf_lpf_step(GRF_Lpf_s *st, float x, float fc, float dt)
{
    if (!st->started) { st->started = 1; st->y = x; return st->y; }
    float alpha = dt / (dt + 1.0f / (6.2831853f * fc));
    st->y += alpha * (x - st->y);
    return st->y;
}

/* VMC.c 的 g(β)（dl/dβ）,腿轴向力反推用 */
static float grf_leg_g(float beta)
{
    float s = sinf(0.5f * beta), c = cosf(0.5f * beta);
    float as = GRF_LEG_A * s;
    return -0.5f * as
           - (as * as * c) / (2.0f * sqrtf(GRF_LEG_B * GRF_LEG_B - as * as));
}

void GRF_Init(void)
{
    grf_dbg = (GRF_Dbg_s){0};
    lpf_fleg_l = lpf_fleg_r = lpf_zdd = lpf_fn_l = lpf_fn_r = (GRF_Lpf_s){0, 0};
    grf_air = 0;
    grf_air_cnt = 0;
}

void GRF_Update(float r_Tf, float r_Tb, float l_Tf, float l_Tb, float dt)
{
    /* ---- 腿轴向力: VMC 逆关系 F = ±(Tb−Tf)/(2g(β)) ----
     * 右腿: Tf=-gF+Tp/2, Tb=+gF+Tp/2  → F=(Tb−Tf)/(2g)
     * 左腿: 电机坐标整体取负(l_VMC_Calc) → F=(Tf−Tb)/(2g)
     * 2026-11-02 静态账验证: 反推值与 Fleg_cmd 吻合 <0.2%。 */
    float g_r = grf_leg_g(r_leg.r_beta);
    float g_l = grf_leg_g(l_leg.l_beta);
    float F_r = 0.0f, F_l = 0.0f;
    if (fabsf(g_r) > 0.01f) F_r = (r_Tb - r_Tf) / (2.0f * g_r);
    if (fabsf(g_l) > 0.01f) F_l = (l_Tf - l_Tb) / (2.0f * g_l);

    /* ---- 竖直分量: θ 为摆杆相对竖直倾角（BalanceObserve 已扣 pitch） ---- */
    float Fz_r = F_r * cosf(ten_balance_observe.r_theta);
    float Fz_l = F_l * cosf(ten_balance_observe.l_theta);

    /* ---- 轮竖直加速度: IMU 比力旋转到竖直方向减 g ----
     * 机体系比力 f_b,竖直分量 f_wz = −sin(p)·ax + cos(p)sin(r)·ay + cos(p)cos(r)·az;
     * roll 修正项为小量(|r|<2°),符号按欧拉 ZYX 惯例,误差可忽略。 */
    float p = chassis_IMU_data->Pitch * 0.01745329252f;
    float r = chassis_IMU_data->Roll   * 0.01745329252f;
    float az_b = chassis_IMU_data->Accel[2];
    float ay_b = chassis_IMU_data->Accel[1];
    float ax_b = chassis_IMU_data->Accel[0];
    float zdd = -sinf(p) * ax_b + cosf(p) * sinf(r) * ay_b
                + cosf(p) * cosf(r) * az_b - GRF_G;

    /* ---- 观测器本体 N = m_w*(z̈+g) + F_leg,z ---- */
    float m = grf_m_w;
    float N_r = m * (zdd + GRF_G) + Fz_r;
    float N_l = m * (zdd + GRF_G) + Fz_l;

    grf_dbg.F_n_l_raw = N_l;
    grf_dbg.F_n_r_raw = N_r;
    grf_dbg.F_leg_l   = grf_lpf_step(&lpf_fleg_l, F_l, grf_fleg_fc, dt);
    grf_dbg.F_leg_r   = grf_lpf_step(&lpf_fleg_r, F_r, grf_fleg_fc, dt);
    grf_dbg.Flegz_l   = grf_dbg.F_leg_l * cosf(ten_balance_observe.l_theta);
    grf_dbg.Flegz_r   = grf_dbg.F_leg_r * cosf(ten_balance_observe.r_theta);
    grf_dbg.zdd_w     = grf_lpf_step(&lpf_zdd, zdd, grf_accel_fc, dt);
    grf_dbg.F_n_l     = grf_lpf_step(&lpf_fn_l, N_l, grf_fleg_fc, dt);
    grf_dbg.F_n_r     = grf_lpf_step(&lpf_fn_r, N_r, grf_fleg_fc, dt);
    grf_air_update(grf_dbg.F_n_l, grf_dbg.F_n_r);
    grf_dbg.air_state = grf_air;
}
