from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
from casadi import SX, vertcat, sin, cos, sqrt

import numpy as np


def gen_model() -> AcadosModel:
    model = AcadosModel()
    model.name = 'model'

    # constants
    M = 0.3  # mass [kg]
    I = 0.004  # inertia [kgm^2]
    L = 0.107  # length of the rod [m]
    Xa = -0.002  # G to conner X a [m]
    Ya = 0.046  # G to conner Y a [m]
    Xb = -0.002  # G to conner X b [m]
    Yb = -0.046  # G to conner Y b [m]
    Xc = 0.084  # G to conner X c [m]
    Yc = -0.046  # G to conner Y c [m]
    K = 0.001  # /s /m to force [Nms]

    # set up states
    v = SX.sym('v')
    w = SX.sym('w')
    th0 = SX.sym('th0')
    th1 = SX.sym('th1')
    th2 = SX.sym('th2')
    th3 = SX.sym('th3')
    th4 = SX.sym('th4')
    th5 = SX.sym('th5')
    model.x = vertcat(v, w, th0, th1, th2, th3, th4, th5)

    # set uo controls
    w0 = SX.sym('w0')
    w1 = SX.sym('w1')
    w2 = SX.sym('w2')
    w3 = SX.sym('w3')
    w4 = SX.sym('w4')
    w5 = SX.sym('w5')
    model.u = vertcat(w0, w1, w2, w3, w4, w5)

    # xdot
    v_dot = SX.sym('v_dot')
    w_dot = SX.sym('w_dot')
    th0_dot = SX.sym('th0_dot')
    th1_dot = SX.sym('th1_dot')
    th2_dot = SX.sym('th2_dot')
    th3_dot = SX.sym('th3_dot')
    th4_dot = SX.sym('th4_dot')
    th5_dot = SX.sym('th5_dot')
    model.xdot = vertcat(v_dot, w_dot, th0_dot, th1_dot, th2_dot, th3_dot,
                         th4_dot, th5_dot)

    # dynamics
    xa = Xa + L * sin(th0)
    ya = Ya + L * sin(th1)
    len_G2a = sqrt(xa * xa + ya * ya)
    fxa = -K * w0 / len_G2a
    fya = -K * w1 / len_G2a

    xb = Xb + L * sin(th2)
    yb = Yb + L * sin(th3)
    len_G2b = sqrt(xb * xb + yb * yb)
    fxb = -K * w2 / len_G2b
    fyb = -K * w3 / len_G2b

    xc = Xc + L * sin(th4)
    yc = Yc + L * sin(th5)
    len_G2c = sqrt(xc * xc + yc * yc)
    fxc = -K * w4 / len_G2c
    fyc = -K * w5 / len_G2c

    f_expl = vertcat(
        (fxa + fxb + fxc) / M,
        (fya * xa - fxa * ya + fyb * xb - fxb * yb + fyc * xc - fxc * yc) / I,
        w0, w1, w2, w3, w4, w5)
    model.f_impl_expr = model.xdot - f_expl
    model.f_expl_expr = f_expl

    return model


N = 200
X_DIM = 8
U_DIM = 6
COST_DIM = 8
COST_E_DIM = COST_DIM - 6
CONSTR_DIM = 12


def gen_ocp() -> AcadosOcp():
    ocp = AcadosOcp()
    ocp.model = gen_model()

    # set dimensions
    ocp.dims.N = N

    # set cost module
    ocp.cost.cost_type = 'NONLINEAR_LS'
    ocp.cost.cost_type_e = 'NONLINEAR_LS'
    QR = np.diag(np.zeros(COST_DIM))
    Q = np.diag(np.zeros(COST_E_DIM))
    ocp.cost.W = QR
    ocp.cost.W_e = Q

    v, w = ocp.model.x[0], ocp.model.x[1]
    w0, w1, w2, w3, w4, w5 = ocp.model.u[0], ocp.model.u[1], ocp.model.u[
        2], ocp.model.u[3], ocp.model.u[4], ocp.model.u[5]

    ocp.cost.yref = np.zeros((COST_DIM, ))
    ocp.cost.yref_e = np.zeros((COST_E_DIM, ))
    # ocp.model.cost_y_expr = vertcat(1 / (v + 1e-6), w, w0, w1, w2, w3, w4, w5)
    ocp.model.cost_y_expr = vertcat(v, w, w0, w1, w2, w3, w4, w5)
    # ocp.model.cost_y_expr_e = vertcat(1 / (v + 1e-6), w)
    ocp.model.cost_y_expr_e = vertcat(v, w)

    # set constraints
    th_min = -0.9
    th_max = 0.9
    ocp.constraints.lbx = np.array(
        [th_min, th_min, th_min, th_min, th_min, th_min])
    ocp.constraints.ubx = np.array(
        [th_max, th_max, th_max, th_max, th_max, th_max])
    ocp.constraints.idxbx = np.array([2, 3, 4, 5, 6, 7])

    # th0, th1, th2, th3, th4, th5 = ocp.model.x[2], ocp.model.x[3], ocp.model.x[
    #     4], ocp.model.x[5], ocp.model.x[6], ocp.model.x[7]

    # ocp.model.con_h_expr = vertcat(th0 - th_min, th_max - th0, th1 - th_min,
    #                                th_max - th1, th2 - th_min, th_max - th2,
    #                                th3 - th_min, th_max - th3, th4 - th_min,
    #                                th_max - th4, th5 - th_min, th_max - th5)

    x0 = np.zeros(X_DIM)
    ocp.constraints.x0 = x0
    # cost_weights = np.zeros(CONSTR_DIM)
    # ocp.cost.zl = cost_weights
    # ocp.cost.Zl = cost_weights
    # ocp.cost.Zu = cost_weights
    # ocp.cost.zu = cost_weights

    # ocp.constraints.lh = np.zeros(CONSTR_DIM)
    # ocp.constraints.uh = 1e4 * np.ones(CONSTR_DIM)
    # ocp.constraints.idxsh = np.arange(CONSTR_DIM)

    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
    ocp.solver_options.integrator_type = 'ERK'
    ocp.solver_options.nlp_solver_type = 'SQP'
    ocp.solver_options.qp_solver_cond_N = 1
    ocp.solver_options.qp_solver_iter_max = 10
    ocp.solver_options.qp_tol = 1e-3

    # set prediction horizon
    T_IDXS = np.linspace(0, 10, N + 1)
    ocp.solver_options.tf = T_IDXS[-1]
    ocp.solver_options.shooting_nodes = T_IDXS
    ocp.solver_options.nlp_solver_ext_qp_res = 1

    return ocp


def main():
    ocp = gen_ocp()
    solver = AcadosOcpSolver(ocp, json_file='acados_ocp.json')

    W = np.asfortranarray(np.diag([1, 1, 1, 1, 1, 1, 1, 1]))
    for i in range(N):
        solver.cost_set(i, 'W', W)
    solver.cost_set(N, 'W', W[:COST_E_DIM, :COST_E_DIM])

    yref = np.zeros((N + 1, COST_DIM))
    yref[:, 0] = 0.01
    for i in range(N):
        solver.cost_set(i, "yref", yref[i])
    solver.cost_set(N, "yref", yref[N][:COST_E_DIM])

    status = solver.solve()
    solver.print_statistics()

    if status != 0:
        print(f'acados returned status {status}.')
        exit()

    X = np.ndarray((N + 1, X_DIM))
    U = np.ndarray((N, U_DIM))
    for i in range(N):
        X[i, :] = solver.get(i, "x")
        U[i, :] = solver.get(i, "u")
    X[N, :] = solver.get(N, "x")
    print(X)
    print(U)


if __name__ == "__main__":
    main()
