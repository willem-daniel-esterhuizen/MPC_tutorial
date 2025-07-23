import numpy as np
from scipy.signal import cont2discrete

A = np.array([[0, 1],[0, 0]])
B = np.array([[0],[1]])
C = np.array([[1, 0],[0, 1]])
D = np.array([[0, 0],[0, 0]])
dt = 0.1 # in seconds
discrete_system = cont2discrete((A, B, C, D), dt, method='zoh')
A_discrete, B_discrete, *_ = discrete_system
dynamics = {'A': A_discrete, 'B': B_discrete}