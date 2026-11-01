"""
在固件状态坐标下设计 LQR。
固件状态 z = [s, ds, yaw, dyaw, θl, dθl, θr, dθr, φ, dφ]（与 LQR.c 列序一致）
固件输入 u = [Tw_l, Tw_r, Tp_l, Tp_r]（与 LQR.c 行序一致），控制律 u = -K (z - z_ref)

关键：固件的 s 来自轮毂编码器，测的是"轮相对小腿(定子)"的转角，
      s_code = x_wheel + R·θ（不是真实轮心位移），yaw 同理用两髋连线定义。
      这里先在物理坐标建模，再精确变换到固件坐标，K 可直接贴入 LQR.c。
"""
import numpy as np
import model_lagrange as ML
from lqrlib import c2d, care, dare, max_delay, delayed_cl
from params import TS, TW_MAX, TP_MAX

STATE_NAMES = ['s', 'ds', 'yaw', 'dyaw', 'thl', 'dthl', 'thr', 'dthr', 'phi', 'dphi']

# ---- 权重（Bryson：Q_ii = 1/σ_i^2，σ = 可接受的最大偏差） ----
SIGMA_X = dict(s=0.28, ds=0.80, yaw=0.50, dyaw=2.0,
               th=0.30, dth=3.0, phi=0.12, dphi=2.5)
SIGMA_U = dict(Tw=0.8, Tp=4.0)


def physical_model(p):
    M, K, Bq = ML.numeric(p)
    Aq = -np.linalg.solve(M, K)
    Bu = np.linalg.solve(M, Bq)
    A = np.block([[np.zeros((5, 5)), np.eye(5)], [Aq, np.zeros((5, 5))]])
    B = np.vstack([np.zeros((5, 4)), Bu])
    return A, B


def to_code(p):
    R, Rl, l = p['R'], p['Rl'], p['l']
    Cq = np.array([
        [0.5, 0.5, R / 2, R / 2, 0],                          # s   = 平均(轮心 + R θ)
        [-1 / (2 * Rl), 1 / (2 * Rl), l / (2 * Rl), -l / (2 * Rl), 0],   # yaw = (x_hr - x_hl)/(2 R_l)
        [0, 0, 1, 0, 0],
        [0, 0, 0, 1, 0],
        [0, 0, 0, 0, 1]])
    P = np.zeros((10, 10))
    for k in range(5):
        P[2 * k, :5] = Cq[k]
        P[2 * k + 1, 5:] = Cq[k]
    return P


def code_model(p):
    A, B = physical_model(p)
    P = to_code(p)
    Pi = np.linalg.inv(P)
    return P @ A @ Pi, P @ B


def weights(sx=None, su=None):
    sx = dict(SIGMA_X, **(sx or {}))
    su = dict(SIGMA_U, **(su or {}))
    s = [sx['s'], sx['ds'], sx['yaw'], sx['dyaw'], sx['th'], sx['dth'],
         sx['th'], sx['dth'], sx['phi'], sx['dphi']]
    Q = np.diag(1 / np.array(s)**2)
    R = np.diag(1 / np.array([su['Tw'], su['Tw'], su['Tp'], su['Tp']])**2)
    return Q, R


def design(p, Q=None, R=None, Ts=TS):
    if Q is None:
        Q, R = weights()
    Ac, Bc = code_model(p)
    Ad, Bd = c2d(Ac, Bc, Ts)
    K, P, res = dare(Ad, Bd, Q, R)
    return dict(K=K, Ac=Ac, Bc=Bc, Ad=Ad, Bd=Bd, res=res, Q=Q, R=R)


def simulate(Ad, Bd, K, z0, T=3.0, Ts=TS, delay=1, sat=True, ext=None):
    """离散线性仿真，含输出限幅与 delay 个采样的指令延迟。ext(k) 可给外部扰动力矩(加在 u 上)。"""
    n = int(T / Ts)
    z = np.array(z0, float)
    buf = [np.zeros(4)] * delay
    Z = np.zeros((n, 10)); U = np.zeros((n, 4))
    lim = np.array([TW_MAX, TW_MAX, TP_MAX, TP_MAX])
    for k in range(n):
        u = -K @ z
        if sat:
            u = np.clip(u, -lim, lim)
        buf.append(u)
        ua = buf.pop(0)
        dist = ext(k) if ext else 0
        z = Ad @ z + Bd @ (ua + dist)
        Z[k] = z; U[k] = u
    return Z, U


def report(p, D, verbose=True):
    K, Ac, Bc, Ad, Bd = D['K'], D['Ac'], D['Bc'], D['Ad'], D['Bd']
    ol = np.linalg.eigvals(Ac)
    clc = np.linalg.eigvals(Ac - Bc @ K)
    cld = np.linalg.eigvals(Ad - Bd @ K)
    md = max_delay(Ad, Bd, K)
    out = dict(ol=ol, clc=clc, maxRe=clc.real.max(), maxabs=np.abs(cld).max(),
               fastest=np.abs(clc).max(), max_delay_ms=md * TS * 1e3)
    tests = {
        'φ 5° (前倾/后仰)': np.eye(10)[8] * np.radians(5),
        'θ 两腿 5°': (np.eye(10)[4] + np.eye(10)[6]) * np.radians(5),
        's 0.3 m': np.eye(10)[0] * 0.3,
        'ds 0.5 m/s (被推)': np.eye(10)[1] * 0.5,
        'yaw 0.3 rad': np.eye(10)[2] * 0.3,
        'dφ 1 rad/s': np.eye(10)[9] * 1.0,
    }
    sims = {}
    for name, z0 in tests.items():
        Z, U = simulate(Ad, Bd, K, z0, delay=1, sat=False)
        sims[name] = dict(Tw=np.abs(U[:, :2]).max(), Tp=np.abs(U[:, 2:]).max(),
                          phi=np.degrees(np.abs(Z[:, 8]).max()),
                          th=np.degrees(np.abs(Z[:, [4, 6]]).max()),
                          s=np.abs(Z[:, 0]).max(),
                          settle=np.abs(Z[-1]).max())
    out['sims'] = sims
    if verbose:
        np.set_printoptions(precision=3, suppress=True, linewidth=160)
        print("开环极点:", np.sort_complex(ol))
        print("闭环极点(连续等效):", np.sort_complex(clc))
        print(f"闭环最大实部 {out['maxRe']:.3f}  最快极点 |λ|={out['fastest']:.1f} rad/s  "
              f"离散 max|z|={out['maxabs']:.5f}  可容忍额外延迟 {out['max_delay_ms']:.0f} ms  DARE残差 {D['res']:.1e}")
        print(f"{'扰动':<16}{'峰值Tw':>8}{'峰值Tp':>8}{'φmax°':>8}{'θmax°':>8}{'s max':>8}{'3s残差':>9}")
        for k, v in sims.items():
            print(f"{k:<16}{v['Tw']:8.2f}{v['Tp']:8.2f}{v['phi']:8.2f}{v['th']:8.2f}{v['s']:8.3f}{v['settle']:9.1e}")
    return out


if __name__ == '__main__':
    p = ML.default_params()
    D = design(p)
    np.set_printoptions(precision=4, suppress=True, linewidth=170)
    print("A_code=\n", D['Ac']); print("B_code=\n", D['Bc'])
    print("K=\n", D['K'])
    report(p, D)
