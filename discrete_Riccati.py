import numpy as np

Q = np.eye(6)
R = np.eye(3)

def dlqr_calculate(A, B, Q, R, returnPE=False):
    '''
    Discrete-time Linear Quadratic Regulator calculation.
    State-feedback control  u[k] = -K*x[k]

    How to apply the function:    
        K = dlqr_calculate(A,B,Q,R)
        K, P, E = dlqr_calculate(A,B,Q,R, return_solution_eigs=True)

    Inputs:
        A, B, Q, R  -> all numpy arrays  (simple float number not allowed)
        returnPE: define as True to return Ricatti solution and final eigenvalues

    Returns:
        K: state feedback gain
        P: Ricatti equation solution
        E: eigenvalues of (A-BK)  (closed loop z-domain poles)
    '''
    from scipy.linalg import solve_discrete_are, inv, eig
    P = solve_discrete_are(A, B, Q, R)  #Ricatti cost
    K = inv(B.T@P@B + R)@B.T@P@A    # K = (B^T P B + R)^-1 B^T P A 

    if returnPE == False:   return K

    from numpy.linalg import eigvals
    eigs = np.array([eigvals(A-B@K)]).T
    return K, P, eigs
