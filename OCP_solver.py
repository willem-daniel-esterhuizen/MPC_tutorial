from casadi import *
import numpy as np
from MPC_tutorial import plot_solution, get_x_next_linear, solve_OCP

if __name__ == "__main__":
    x_init = np.array([[0.5],[0.5]]) # 2 x 1 vector
    K = 10
    number_of_iterations = 150

    u_cl = np.zeros((1, number_of_iterations))
    x_cl = np.zeros((2, number_of_iterations + 1))
    x_cl[:, 0] = x_init[:, 0]

    x_hat = x_init
    for i in range(number_of_iterations):
        _, u_opt = solve_OCP(x_hat, K)
        u_opt_first_element = u_opt[0]

        # save closed loop x and u
        u_cl[:, i] = u_opt_first_element
        x_next = get_x_next_linear(x_hat, u_opt_first_element)
        x_cl[:, i+1] = np.squeeze(x_next)

        # update initial state
        x_hat = x_next

    plot_solution(x_cl, u_cl)