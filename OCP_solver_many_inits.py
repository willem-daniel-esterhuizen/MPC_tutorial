from casadi import *
import numpy as np
from MPC_tutorial import get_x_next_linear, plot_solution_hold_on, solve_OCP, plot_constraints
import matplotlib.pyplot as plt

if __name__ == "__main__":
    fig, ax = plt.subplots()
    x1_init_min = -0.6
    x1_init_max = 0.2
    x2_init_min = -0.6
    x2_init_max = 0.6

    for x1_init in np.arange(x1_init_min, 0.6, x1_init_max):
        for x2_init in [x2_init_min, x2_init_max]:
            x_init = np.array([[x1_init],[x2_init]]) # 2 x 1 vector

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

            plot_solution_hold_on(ax, x_cl)
            
            x_1_max = 1
            x_1_min = -1
            plot_constraints(ax, x_1_max, x_1_min, x2_init_min, x2_init_max)

plt.show()