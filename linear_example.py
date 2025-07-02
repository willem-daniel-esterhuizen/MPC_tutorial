import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import cont2discrete

def plot_solution(x_tot, u):
    fig = plt.figure(figsize=(10, 8))
    gs = fig.add_gridspec(3, 1)
    ax1 = fig.add_subplot(gs[0])
    ax2 = fig.add_subplot(gs[1], sharex=ax1)
    ax3 = fig.add_subplot(gs[2])

    # Plot state versus time
    time = np.arange(len(x_tot) + 1)
    x_1 = [x[0][0] for x in x_tot]
    x_2 = [x[1][0] for x in x_tot]

    x_1 = np.append(x_1[0], x_1)
    x_2 = np.append(x_2[0], x_2)

    ax1.step(time, x_1, 'b-', label='Position')
    ax1.step(time, x_2, 'r-', label='Velocity')
    ax1.set_xlabel('Time step')
    ax1.set_ylabel('State')
    ax1.legend()
    ax1.grid(True)
    
    # Plot control versus time
    time = np.arange(len(u) + 1)
    ax2.step(time, np.append(u[0], u), 'g-', label='Control input')
    ax2.set_xlabel('Time step')
    ax2.set_ylabel('Control input')
    ax2.legend()
    ax2.grid(True)

    # Plot the state in state space
    ax3.plot([x[0][0] for x in x_tot], [x[1][0] for x in x_tot], 'k-', label='State')
    ax3.set_xlabel('Position')
    ax3.set_ylabel('Velocity')
    ax3.legend()
    ax3.grid(True)
    
    plt.tight_layout()
    plt.show()
    plt.show()

# get the discrete-time system due to zero-order-hold control
A = np.array([[0, 1],[0, 0]])
B = np.array([[0],[1]])
C = np.array([[1, 0],[0, 1]])
D = np.array([[0, 0],[0, 0]])
dt = 0.1 # in seconds
discrete_system= cont2discrete((A, B, C, D), dt, method='zoh')
A_discrete, B_discrete, *_ = discrete_system


x_0 = np.array([[1],
                [1]])
u = np.array([3, 4, 5, 6])

x_k = x_0
x_tot = [x_k]
for k in range(len(u)):
    u_k = u[k]
    x_next = np.dot(A, x_k) + np.dot(B, u_k)
    x_tot.append(x_next)

    # update
    x_k = x_next

plot_solution(x_tot, u)

