from casadi import *
import numpy as np
from MPC_tutorial import plot_solution

def get_x_next_linear(x_current, u_current):
    A = np.array([[1. , 0.1],
                  [0. , 1. ]])
    B = np.array([[0.005],
                  [0.1  ]])
    x_next = A@x_current + B@u_current # @'s are for matrix multiplication, *'s are for element-wise multiplication
    return x_next

def get_x_total(x_hat, u_tot, get_x_next_symbolic_func):
    x_k = x_hat
    x_tot = [x_k]
    for k in range(u_tot.shape[1]):
        u_k = u_tot[:,[k]] # the k'th column
        x_k = get_x_next_symbolic_func(x_k, u_k)
        x_tot.append(x_k)
    return horzcat(*x_tot)

def get_objective(x_hat, u_tot, get_x_total_symbolic_func):
    x_tot = get_x_total_symbolic_func(x_hat, u_tot)

    # # TEST
    # x_tot = np.array([[1, 2], [3, 4], [5, 6]])
    # u_tot = np.array([3, 4]).reshape(1,-1)
    objective = 0
    for k in range(u_tot.shape[0]):
        objective += u_tot[:,[k]].T@u_tot[:,[k]] + x_tot[:,[k]].T@x_tot[:,[k]] # running cost
    
    # final cost
    objective += x_tot[:,[-1]].T@x_tot[:,[-1]]
    return objective


def solve_OCP(x_hat_numeric, u_numeric):
    # get state and control dimensions
    n = x_hat_numeric.shape[0] # state dimension
    m = u_numeric.shape[0] # control dimension
    K = u_numeric.shape[1] # horizon length

    # set up casadi symbolic functions
    x_current = MX.sym('x_current', n, 1)
    u_current = MX.sym('u_current', m, 1)
    get_x_next_symbolic_func = Function('get_x_next_symbolic_func', [x_current, u_current], [get_x_next_linear(x_current, u_current)])

    x_hat = MX.sym('x_hat', n, 1)
    u_tot = MX.sym('u', m, K)
    get_x_total_symbolic_func = Function('get_x_total_symbolic_func', [x_hat, u_tot], [get_x_total(x_hat, u_tot, get_x_next_symbolic_func)])
    get_objective_symbolic_func = Function('get_objective_symbolic_func', [x_hat, u_tot], [get_objective(x_hat, u_tot, get_x_total_symbolic_func)])

    # x_tot = get_x_total_symbolic_func(x_hat_numeric, u_numeric)

    objective = get_objective_symbolic_func(x_hat, u_tot)

    'stop'

#     *** FROM HERE NEXT TIME ***
# x = SX.sym('x'); y = SX.sym('y'); z = SX.sym('z')
# nlp = {'x':vertcat(x,y,z), 'f':x**2+100*z**2, 'g':z+(1-x)**2-y}
# S = nlpsol('S', 'ipopt', nlp)
    # x_tot = MX.sym('x_tot', n*(K+1))
    # get_objective_symbolic_func = Function('get_objective_symbolic_func', [x_tot, u], get_objective(x_tot, u))

    return 1
    # return get_x_total_symbolic_func(x_hat_numeric, u_numeric)

if __name__ == "__main__":
    x_hat_numeric = np.array([[1],[1]]) # 2 x 1 vector
    u_numeric = np.array([-10, -10, -10, 5, 5, 5]).reshape(1,-1) # 1 x 6 vector

    x_tot_numeric = solve_OCP(x_hat_numeric, u_numeric)



    # PLOTTING

    n = x_hat_numeric.shape[0]
    m = u_numeric.shape[0]
    x_tot_numeric_list = [np.array(x).reshape(n,1) for x in x_tot_numeric] # convert to n x (K+1)
    u_numeric_list = [np.array(u) for u in u_numeric[0]]
    plot_solution(x_tot_numeric_list, u_numeric_list)


# from casadi import *
# import numpy as np
# from MPC_tutorial import plot_solution

# def get_x_next_linear(x_current, u_current):
#     A = np.array([[1. , 0.1],
#                   [0. , 1. ]])
#     B = np.array([[0.005],
#                   [0.1  ]])
#     x_next = A@x_current + B@u_current # @'s are for matrix multiplication, *'s are for element-wise multiplication
#     return x_next

# def get_x_total(x_hat, u_tot, get_x_next_symbolic_func):
#     x_k = x_hat
#     x_tot = [x_k]
#     for k in range(u_tot.shape[1]):
#         u_k = u_tot[k]
#         x_k = get_x_next_symbolic_func(x_k, u_k)
#         x_tot.append(x_k)
#     return x_tot

# def solve_IVP(x_hat_numeric, u_numeric):
#     # get state and control dimensions
#     n = x_hat_numeric.shape[0] # state dimension
#     m = u_numeric.shape[0] # control dimension
#     K = u_numeric.shape[1] # horizon length

#     # set up casadi symbolic functions
#     x_current = MX.sym('x_current', n, 1)  # n x 1
#     u_current = MX.sym('u_current', m, 1)  # m x 1
#     get_x_next_symbolic_func = Function('get_x_next_symbolic_func', [x_current, u_current], [get_x_next_linear(x_current, u_current)])

#     x_hat = MX.sym('x_hat', n, 1) # n x 1
#     u_tot = MX.sym('u_tot', m, K) # m x K

#     get_x_total_symbolic_func = Function('get_x_total_symbolic_func', [x_hat, u_tot], get_x_total(x_hat, u_tot, get_x_next_symbolic_func))
    
#     return get_x_total_symbolic_func(x_hat_numeric, u_numeric)

# if __name__ == "__main__":

#     # # TEST
#     # x_hat_numeric = np.array([[1],[2]]) # n x 1 vector
#     # u_numeric = np.array([3, 4]).reshape(1,-1) # m x K array
#     # x_tot_numeric = solve_IVP(x_hat_numeric, u_numeric)

#     # A = np.array([[1. , 0.1],
#     #             [0. , 1. ]])
#     # B = np.array([[0.005],
#     #               [0.1  ]])
    
#     # x_expected = [x_hat_numeric, A@x_hat_numeric + B*u_numeric[0, 0], A@(A@x_hat_numeric + B*u_numeric[0, 0]) + B*u_numeric[0, 1]]

#     x_hat_numeric = np.array([[1],[1]]) # 2 x 1 vector
#     u_numeric = np.array([-10, -10, -10, 5, 5, 5]).reshape(1,-1) # 1 x K vector

#     x_tot_numeric = solve_IVP(x_hat_numeric, u_numeric)

#     n = x_hat_numeric.shape[0]
#     m = u_numeric.shape[0]    
#     x_tot_numeric_list = [np.array(x).reshape(n,1) for x in x_tot_numeric] # convert to n x (K+1)
#     u_numeric_list = [np.array(u) for u in u_numeric[0]]
#     plot_solution(x_tot_numeric_list, u_numeric_list)    