import numpy as np
from scipy.signal import cont2discrete
from MPC_tutorial import plot_solution




# A_discrete
# array([[1. , 0.1],
#        [0. , 1. ]])
# B_discrete
# array([[0.005],
#        [0.1  ]])

x_0 = np.array([[1],
                [1]])
x_hat = x_0

x = SX.sym('x'); y = SX.sym('y'); z = SX.sym('z')
nlp = {'x':vertcat(x,y,z), 'f':x**2+100*z**2, 'g':z+(1-x)**2-y}
S = nlpsol('S', 'ipopt', nlp)
print(S)

number_of_iterations = 10
for i in range(number_of_iterations):
    u_star = solve_OCP(x_hat, dynamics)


u = np.array([3, 4, 5, 6])

x_k = x_0
x_tot = [x_k]
for k in range(len(u)):
    u_k = u[k]
    x_next = np.dot(A_discrete, x_k) + np.dot(B_discrete, u_k)
    x_tot.append(x_next)

    # update
    x_k = x_next

plot_solution(x_tot, u)

# # get the discrete-time system due to zero-order-hold control
# A = np.array([[0, 1],[0, 0]])
# B = np.array([[0],[1]])
# C = np.array([[1, 0],[0, 1]])
# D = np.array([[0, 0],[0, 0]])
# dt = 0.1 # in seconds
# discrete_system = cont2discrete((A, B, C, D), dt, method='zoh')
# A_discrete, B_discrete, *_ = discrete_system
# dynamics = {'A': A_discrete, 'B': B_discrete}