import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator
import numpy as np
from casadi import *

def plot_solution(x_tot, u):
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
    time = np.arange(len(u) + 1)
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
    plt.show()