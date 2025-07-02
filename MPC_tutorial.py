import matplotlib.pyplot as plt
import numpy as np

def plot_solution(x_tot, u_tot):
    # Create figure with 2 subplots vertically stacked
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
    
    # Plot states in first subplot
    time = np.arange(len(x_tot))
    ax1.plot(time, [x[0][0] for x in x_tot], 'b-', label='Position')
    ax1.plot(time, [x[1][0] for x in x_tot], 'r--', label='Velocity')
    ax1.set_xlabel('Time step')
    ax1.set_ylabel('States')
    ax1.legend()
    ax1.grid(True)
    
    # Plot control inputs in second subplot
    ax2.step(time[:-1], u_tot, 'g-', label='Control input')
    ax2.set_xlabel('Time step')
    ax2.set_ylabel('Control input')
    ax2.legend()
    ax2.grid(True)
    
    plt.tight_layout()
    plt.show()
    plt.show()