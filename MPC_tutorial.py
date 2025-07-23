import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator
import numpy as np
from casadi import *

def solve_OCP(x_hat, K):
    n = 2 # state dimension
    m = 1 # control dimension

    # Constraints for all k
    u_max = 1
    x_1_max = 1
    x_1_min = -1

    # Linear cost matrices
    Q = np.array([[1. , 0],
                [0. , 1. ]])
    R = np.array([[1]])
    Q_K = Q

    opti = Opti()
    x_tot = opti.variable(n, K+1)  # State trajectory
    u_tot = opti.variable(m, K)    # Control trajectory

    # Specify the initial condition
    opti.subject_to(x_tot[:, 0] == x_hat)

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

    # constrain the position
    opti.subject_to(opti.bounded(x_1_min, x_tot[0,:], x_1_max))

    # Say we want to minimise the cost and specify the solver (ipopt)
    opts = {"ipopt.print_level": 0, "print_time": 0}
    opti.minimize(cost)
    opti.solver("ipopt", opts)
    
    solution = opti.solve()

    # Get solution
    x_opt = solution.value(x_tot)
    u_opt = solution.value(u_tot)

    # plot_solution(x_opt, u_opt.reshape(1,-1))

    return x_opt, u_opt

def get_x_next_linear(x, u):
    # Linear system
    A = np.array([[1. , 0.1],
                [0. , 1. ]])
    B = np.array([[0.005],
                  [0.1  ]])
    
    return mtimes(A, x) + mtimes(B, u)

def plot_constraints(ax, x_1_max, x_1_min, x2_init_min, x2_init_max):
    ax.plot([x_1_min, x_1_min], [x2_init_min, x2_init_max], 'k-')
    ax.plot([x_1_max, x_1_max], [x2_init_min, x2_init_max], 'k-')


def plot_solution_hold_on(ax, x_tot):
    x_1 = x_tot[0]
    x_2 = x_tot[1]

    x_1 = np.append(x_1[0], x_1)
    x_2 = np.append(x_2[0], x_2)

    ax.plot(x_1, x_2, 'b-')
    ax.plot([x_1[0], x_1[-1]], [x_2[0], x_2[-1]], 'k.')
    ax.set_xlabel('Position')
    ax.set_ylabel('Velocity')
    ax.grid(True)

def plot_solution(x_tot, u_tot):
    # this takes lists of numpy arrays...
    fig = plt.figure(figsize=(10, 8))
    gs = fig.add_gridspec(3, 1)
    ax1 = fig.add_subplot(gs[0])
    ax2 = fig.add_subplot(gs[1], sharex=ax1)
    ax3 = fig.add_subplot(gs[2])

    # Plot state versus time
    x_1 = x_tot[0]
    x_2 = x_tot[1]

    # Duplicate the initial state so we can have a nice bar plots.
    x_1 = np.append(x_1[0], x_1)
    x_2 = np.append(x_2[0], x_2)
    time = np.arange(x_tot.shape[1] + 1) # because we duplicate the initial state

    ax1.step(time, x_1, 'b-', label='Position')
    ax1.step(time, x_2, 'r-', label='Velocity')
    ax1.set_xlabel('Time step')
    ax1.xaxis.set_major_locator(MaxNLocator(integer=True))
    ax1.set_ylabel('State')
    ax1.legend()
    ax1.grid(True)
    
    # Plot control versus time
    u = u_tot[0]
    time = np.arange(u_tot.shape[1] + 1)
    ax2.step(time, np.append(u[0], u), 'g-', label='Control input')
    ax2.set_xlabel('Time step')
    ax2.set_ylabel('Control input')
    ax2.legend()
    ax2.grid(True)

    # Plot the state in state space
    ax3.plot(x_1, x_2, 'k-', label='State')
    ax3.set_xlabel('Position')
    ax3.set_ylabel('Velocity')
    ax3.legend()
    ax3.grid(True)
    
    # Plot the initial and final states
    ax3.plot([x_1[0], x_1[-1]], [x_2[0], x_2[-1]], 'k.')

    plt.tight_layout()
    plt.show()