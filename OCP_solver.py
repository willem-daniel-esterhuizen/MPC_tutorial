from casadi import *
import numpy as np
from MPC_tutorial import plot_solution

def get_x_next_linear(x, u):
    # Linear system
    A = np.array([[1. , 0.1],
                [0. , 1. ]])
    B = np.array([[0.005],
                  [0.1  ]])
    
    return mtimes(A, x) + mtimes(B, u)

def solve_OCP(x_init, K):
    n = 2 # state dimension
    m = 1 # control dimension

    # Constraints
    u_max = 1
    x_1_max = 1.5
    x_1_min = -10

    # Linear cost matrices
    Q = np.array([[1. , 0],
                [0. , 1. ]])
    R = np.array([[1]])
    Q_K = Q

    opti = Opti()
    x_tot = opti.variable(n, K+1)  # State trajectory
    u_tot = opti.variable(m, K)    # Control trajectory

    # Specify the initial condition
    opti.subject_to(x_tot[:, 0] == x_init)

    cost = 0
    for k in range(K):
        # add dynamic constraints
        x_tot_next = get_x_next_linear(x_tot[:, k], u_tot[:, k])
        opti.subject_to(x_tot[:, k+1] == x_tot_next)

        # add to the cost
        cost += mtimes([x_tot[:,k].T, Q, x_tot[:,k]]) + mtimes([u_tot[:,k].T, R, u_tot[:,k]])

    cost += mtimes([x_tot[:,K].T, Q_K, x_tot[:,K]])

    # constrain the control
    opti.subject_to(opti.bounded(-u_max, u_tot, u_max))

    # constrain the state
    opti.subject_to(opti.bounded(x_1_min, x_tot[0,:], x_1_max))

    # Say we want to minimise the cost and specify the solver (ipopt)
    opti.minimize(cost)
    opti.solver("ipopt")
    
    solution = opti.solve()

    # Get solution
    x_opt = solution.value(x_tot)
    u_opt = solution.value(u_tot)

    # print(opti.debug.show_infeasibilities())
    # x_guess = opti.debug.value(x_tot)
    # u_guess = opti.debug.value(u_tot)

    # plot_solution(x_opt, u_opt)

    return x_opt, u_opt

if __name__ == "__main__":
    x_hat = np.array([[1],[1]]) # 2 x 1 vector
    K = 10
    number_of_iterations = 10

    u_cl = []
    for i in range(number_of_iterations):
        _, u_opt = solve_OCP(x_hat, K)
        ## *** NEXT TIME FROM HERE! ****
        # UPDATE x_hat here
        u_cl.append(u_opt[0])

    # get the closed loop state
    x_cl = np.zeros((2, number_of_iterations + 1))
    x_cl[:, 0] = x_hat[0]

    for i in range(number_of_iterations):
        x_cl[:, i+1] = np.squeeze(get_x_next_linear(x_cl[:, i], u_cl[i]))

    plot_solution(x_cl, u_cl)
    'stop'