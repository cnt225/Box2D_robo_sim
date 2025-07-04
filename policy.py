# policy.py
import numpy as np
import cvxpy as cp

def potential_field_policy(links, target, obstacles):
    """
    각 링크 말단 위치에 대해 attractive + repulsive 벡터 계산,
    J^T을 이용해 joint torque(또는 velocity)로 변환 후 반환.
    """
    # 1) End-effector 좌표
    end = links[-1].worldCenter
    p_end = np.array(end)

    # 2) Attraction force toward target
    F_att = 5*(np.array(target) - p_end)

    # 3) Repulsion from each obstacle
    F_rep = np.zeros(2)
    for obs in obstacles:
        p_obs = np.array(obs.position)
        v = p_end - p_obs
        d = np.linalg.norm(v)
        if d < 1.0:
            F_rep += 2*(1.0/d - 1.0)* (v/d**3)

    F_total = F_att + F_rep

    # 4) Jacobian 계산은 main.py 에서 처리하고,
    #    여기서는 그냥 반환
    return F_total  # planar => (Fx, Fy)

def rmp_policy(links, target, obstacles):
    # 1) end-effector 위치
    p_ee = np.array(links[-1].worldCenter)

    # 2) goal attractor RMP: f = -k·(p - target), M = w·I
    k_att, w_att = 10.0, 1.0
    f_att = -k_att * (p_ee - target)
    M_att = w_att * np.eye(2)

    # 3) obstacle repeller RMPs (only at ee)
    f_rep = np.zeros(2)
    M_rep = np.zeros((2,2))
    for obs in obstacles:
        p_obs = np.array(obs.position)
        v = p_ee - p_obs
        d = np.linalg.norm(v)
        if d < 1.5:
            w = 1.0 / (d**4)
            f_rep += w * (v / (d**2))
            M_rep += np.eye(2) * w

    # 4) combine: M_tot = M_att + M_rep, f_tot = f_att + f_rep
    M_tot = M_att + M_rep
    f_tot = f_att + f_rep

    # 5) RMP acceleration: a = M_inv @ f
    try:
        M_inv = np.linalg.inv(M_tot + 1e-6 * np.eye(2))  # 수치적 안정성을 위해 작은 값 추가
        acceleration = M_inv @ f_tot
        
        # acceleration을 force로 변환 (간단히 스케일링)
        F_total = acceleration * 5.0  # 적절한 스케일 팩터
        
    except np.linalg.LinAlgError:
        # singular matrix인 경우 potential field로 fallback
        F_total = -k_att * (p_ee - target)
    
    return F_total  # potential_field_policy와 동일한 형태로 반환

def cbf_qp_policy(links, target, obstacles):
    p_ee = np.array(links[-1].worldCenter)
    v_nom = 2*(target - p_ee)  # nominal twist

    # Jacobian J2 (2×n) 은 main.py에서 미리 계산했다고 가정
    J2 = compute_jacobian2(links)  # 사용자 정의

    # 변수: joint velocity u (n,)
    u = cp.Variable(J2.shape[1])
    # objective: ||J2 u - v_nom||^2
    obj = cp.Minimize(cp.sum_squares(J2@u - v_nom))
    # constraints: h(x) = ||p_ee - p_obs||^2 - r^2 >= 0 →  
    # ∇h·(J2 u) + α h >= 0
    cons = []
    alpha = 10.0
    for obs in obstacles:
        p_obs = np.array(obs.position)
        h = np.linalg.norm(p_ee - p_obs)**2 - 0.3**2
        grad_h = 2*(p_ee - p_obs)
        cons.append(grad_h @ (J2@u) + alpha*h >= 0)
    prob = cp.Problem(obj, cons)
    prob.solve(solver=cp.OSQP, warm_start=True)

    # 반환: joint velocity → main.py에서 torque로 바꿔 적용
    return u.value