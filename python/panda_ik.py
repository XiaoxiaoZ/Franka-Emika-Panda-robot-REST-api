"""Analytical inverse kinematics for the Franka Emika Panda.

Port of Yanhao He's franka_analytical_ik (He & Liu, "Analytical Inverse
Kinematics for Franka Emika Panda", 2021). The Panda has 7 joints, so a 6-D
tool pose leaves one degree of redundancy; this solver takes the wrist roll
q7 as that free parameter and returns up to 4 closed-form solutions for it.
Sweeping q7 therefore enumerates the whole self-motion manifold, which lets
the caller pick the solution closest to the current configuration instead of
taking whatever a numerical solver drifts to (KDL routinely parks q7 on its
+-2.8973 limit).

All poses are 4x4 homogeneous matrices of the Franka EE frame (flange +
0.1034 m along z, rotated -45 deg about z -- ``panda_EE`` / ``panda_hand_tcp``
in the URDF) expressed in ``panda_link0``.
"""
import math
import numpy as np

Q_MIN = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
Q_MAX = np.array([ 2.8973,  1.7628,  2.8973, -0.0698,  2.8973,  3.7525,  2.8973])

# Link geometry (m) and derived constants, from the paper / reference code.
_D1, _D3, _D5, _D7E, _A4, _A7 = 0.3330, 0.3160, 0.3840, 0.2104, 0.0825, 0.0880
_LL24, _LL46 = 0.10666225, 0.15426225
_L24, _L46 = 0.326591870689, 0.392762332715
_THETA_H46, _THETA_342, _THETA_46H = 1.35916951803, 1.31542071191, 0.211626808903

# Modified-DH parameters (Craig) for the forward-kinematics check.
_DH_A = [0, 0, 0, 0.0825, -0.0825, 0, 0.088, 0]
_DH_D = [0.333, 0, 0.316, 0, 0.384, 0, 0, 0.107]
_DH_ALPHA = [0, -math.pi / 2, math.pi / 2, math.pi / 2, -math.pi / 2, math.pi / 2, math.pi / 2, 0]


def _unit(v):
    return v / np.linalg.norm(v)


def ik(O_T_EE, q7, q_actual):
    """Up to 4 joint solutions for tool pose ``O_T_EE`` (4x4) at wrist roll
    ``q7``. Entries that are unreachable / outside joint limits are None.
    ``q_actual`` is only used to disambiguate q1 at the shoulder singularity."""
    T = np.asarray(O_T_EE, dtype=float)
    none4 = [None, None, None, None]
    if q7 <= Q_MIN[6] or q7 >= Q_MAX[6]:
        return none4
    q_all = [np.full(7, np.nan) for _ in range(4)]
    for q in q_all:
        q[6] = q7

    z_EE = T[:3, 2]
    p_EE = T[:3, 3]
    p_7 = p_EE - _D7E * z_EE
    x_EE_6 = np.array([math.cos(q7 - math.pi / 4), -math.sin(q7 - math.pi / 4), 0.0])
    x_6 = _unit(T[:3, :3] @ x_EE_6)
    p_6 = p_7 - _A7 * x_6

    # q4 from the triangle joint2 - joint4 - joint6
    p_2 = np.array([0.0, 0.0, _D1])
    V26 = p_6 - p_2
    LL26 = float(V26 @ V26)
    L26 = math.sqrt(LL26)
    if _L24 + _L46 < L26 or _L24 + L26 < _L46 or L26 + _L46 < _L24:
        return none4
    theta246 = math.acos((_LL24 + _LL46 - LL26) / 2.0 / _L24 / _L46)
    q4 = theta246 + _THETA_H46 + _THETA_342 - 2.0 * math.pi
    if q4 <= Q_MIN[3] or q4 >= Q_MAX[3]:
        return none4
    for q in q_all:
        q[3] = q4

    # q6 (two branches)
    theta462 = math.acos((LL26 + _LL46 - _LL24) / 2.0 / L26 / _L46)
    theta26H = _THETA_46H + theta462
    D26 = -L26 * math.cos(theta26H)
    Z_6 = np.cross(z_EE, x_6)
    Y_6 = np.cross(Z_6, x_6)
    R_6 = np.column_stack((x_6, _unit(Y_6), _unit(Z_6)))
    V_6_62 = R_6.T @ (-V26)
    Phi6 = math.atan2(V_6_62[1], V_6_62[0])
    arg = D26 / math.sqrt(V_6_62[0] ** 2 + V_6_62[1] ** 2)
    if abs(arg) > 1.0:
        return none4
    Theta6 = math.asin(arg)
    q6 = [math.pi - Theta6 - Phi6, Theta6 - Phi6]
    valid6 = [True, True]
    for i in range(2):
        if q6[i] <= Q_MIN[5]:
            q6[i] += 2.0 * math.pi
        elif q6[i] >= Q_MAX[5]:
            q6[i] -= 2.0 * math.pi
        if q6[i] <= Q_MIN[5] or q6[i] >= Q_MAX[5]:
            valid6[i] = False
        else:
            q_all[2 * i][5] = q6[i]
            q_all[2 * i + 1][5] = q6[i]
    if not any(valid6):
        return none4

    # q1 & q2
    thetaP26 = 3.0 * math.pi / 2 - theta462 - theta246 - _THETA_342
    thetaP = math.pi - thetaP26 - theta26H
    LP6 = L26 * math.sin(thetaP26) / math.sin(thetaP)
    z_5_all = [None] * 4
    V2P_all = [None] * 4
    for i in range(2):
        if not valid6[i]:
            continue
        z_6_5 = np.array([math.sin(q6[i]), math.cos(q6[i]), 0.0])
        z_5 = R_6 @ z_6_5
        V2P = p_6 - LP6 * z_5 - p_2
        z_5_all[2 * i] = z_5_all[2 * i + 1] = z_5
        V2P_all[2 * i] = V2P_all[2 * i + 1] = V2P
        L2P = np.linalg.norm(V2P)
        if abs(V2P[2] / L2P) > 0.999:
            # shoulder singularity: q1 is free, keep the current one
            q_all[2 * i][0] = q_all[2 * i + 1][0] = q_actual[0]
            q_all[2 * i][1] = q_all[2 * i + 1][1] = 0.0
        else:
            q_all[2 * i][0] = math.atan2(V2P[1], V2P[0])
            q_all[2 * i][1] = math.acos(V2P[2] / L2P)
            q_all[2 * i + 1][0] = q_all[2 * i][0] + (math.pi if q_all[2 * i][0] < 0 else -math.pi)
            q_all[2 * i + 1][1] = -q_all[2 * i][1]

    out = []
    for i in range(4):
        q = q_all[i]
        if V2P_all[i] is None or np.isnan(q[0]) or \
                q[0] <= Q_MIN[0] or q[0] >= Q_MAX[0] or q[1] <= Q_MIN[1] or q[1] >= Q_MAX[1]:
            out.append(None)
            continue
        # q3
        z_3 = _unit(V2P_all[i])
        Y_3 = -np.cross(V26, V2P_all[i])
        y_3 = _unit(Y_3)
        x_3 = np.cross(y_3, z_3)
        c1, s1 = math.cos(q[0]), math.sin(q[0])
        R_1 = np.array([[c1, -s1, 0.0], [s1, c1, 0.0], [0.0, 0.0, 1.0]])
        c2, s2 = math.cos(q[1]), math.sin(q[1])
        R_1_2 = np.array([[c2, -s2, 0.0], [0.0, 0.0, 1.0], [-s2, -c2, 0.0]])
        R_2 = R_1 @ R_1_2
        x_2_3 = R_2.T @ x_3
        q[2] = math.atan2(x_2_3[2], x_2_3[0])
        if q[2] <= Q_MIN[2] or q[2] >= Q_MAX[2]:
            out.append(None)
            continue
        # q5
        VH4 = p_2 + _D3 * z_3 + _A4 * x_3 - p_6 + _D5 * z_5_all[i]
        c6, s6 = math.cos(q[5]), math.sin(q[5])
        R_5_6 = np.array([[c6, -s6, 0.0], [0.0, 0.0, -1.0], [s6, c6, 0.0]])
        R_5 = R_6 @ R_5_6.T
        V_5_H4 = R_5.T @ VH4
        q[4] = -math.atan2(V_5_H4[1], V_5_H4[0])
        if q[4] <= Q_MIN[4] or q[4] >= Q_MAX[4]:
            out.append(None)
            continue
        out.append(q)
    return out


def fk(q):
    """Pose (4x4) of the Franka EE frame (panda_hand_tcp) in panda_link0."""
    T = np.eye(4)
    for i in range(8):
        th = q[i] if i < 7 else 0.0
        a, d, al = _DH_A[i], _DH_D[i], _DH_ALPHA[i]
        ct, st, ca, sa = math.cos(th), math.sin(th), math.cos(al), math.sin(al)
        T = T @ np.array([[ct, -st, 0, a],
                          [st * ca, ct * ca, -sa, -d * sa],
                          [st * sa, ct * sa, ca, d * ca],
                          [0, 0, 0, 1]])
    c, s = math.cos(-math.pi / 4), math.sin(-math.pi / 4)
    T_8_EE = np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0.1034], [0, 0, 0, 1]])
    return T @ T_8_EE


def pose_error(T_a, T_b):
    """(position error [m], orientation error [rad]) between two poses."""
    dp = float(np.linalg.norm(T_a[:3, 3] - T_b[:3, 3]))
    R = T_a[:3, :3].T @ T_b[:3, :3]
    c = max(-1.0, min(1.0, (np.trace(R) - 1.0) / 2.0))
    return dp, math.acos(c)


def solutions_near(O_T_EE, q_current, weights=(1.0,) * 7, limit_margin=0.05,
                   q7_step=0.02, pos_tol=1e-3, rot_tol=math.radians(0.5)):
    """Every FK-verified, limit-clear IK solution for ``O_T_EE``, sweeping
    q7 over its range, sorted by ``max(weights * |q - q_current|)``.
    Returns a list of (cost, q) with q a 7-list."""
    T = np.asarray(O_T_EE, dtype=float)
    qc = np.asarray(q_current, dtype=float)
    w = np.asarray(weights, dtype=float)
    lo, hi = Q_MIN + limit_margin, Q_MAX - limit_margin
    found = []
    q7 = lo[6]
    while q7 <= hi[6]:
        for q in ik(T, q7, qc):
            if q is None or np.any(np.isnan(q)):
                continue
            if np.any(q <= lo) or np.any(q >= hi):
                continue
            dp, dr = pose_error(fk(q), T)
            if dp > pos_tol or dr > rot_tol:
                continue
            cost = float(np.max(w * np.abs(q - qc)))
            found.append((cost, [float(v) for v in q]))
        q7 += q7_step
    found.sort(key=lambda t: t[0])
    return found
