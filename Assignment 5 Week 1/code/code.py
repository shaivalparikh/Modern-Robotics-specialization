import csv
import numpy as np
import scipy.optimize
import math
import pdb

def form_closure(contacts,normal):

    # initialize linear program arrays
    F = np.zeros((3,np.shape(contacts)[1]))
    A = -np.eye(np.shape(contacts)[1])
    b = -np.ones(np.shape(contacts)[1])
    f = np.ones(np.shape(contacts)[1])

    # build F matrix
    for ii in range(0,np.shape(contacts)[1]):
        norm_v = np.array([np.cos(normal[ii]),np.sin(normal[ii]),0])
        F[0,ii] = np.cross(contacts[:,ii],norm_v)[2]
        F[1:3,ii] = norm_v[0:2]

    # compute linear program
    sol = scipy.optimize.linprog(f, A_ub = A, b_ub = b,A_eq = F, b_eq = np.array([0,0,0]))

    # return success or fail
    if sol.success == True:
        return (sol.success,sol.x)
    else:
        return (sol.success,np.array([]))
    
if __name__ == "__main__":

    # figure 1
    print('\nFigure 1')
    contacts = np.array([[-1,0,-2,-1],[0,1,1,2]])
    normal = np.array([np.pi,np.pi/2,0,-np.pi/2])
    
    print('\nInput:')
    print('contacts: \n%s' % contacts)
    print('normal: \n%s' % normal)

    # compute solution
    (success,k) = form_closure(contacts,normal)

    print('\nOutput:')
    print('success: \n%s' % success)
    print('k: \n%s' % k)

    # figure 2
    print('\nFigure 2')
    contacts = np.array([[-1,0,1,-1],[0,1,-2,2]])
    normal = np.array([np.pi,np.pi/2,np.pi/2,-np.pi/2])
    
    print('\nInput:')
    print('contacts: \n%s' % contacts)
    print('normal: \n%s' % normal)

    # compute solution
    (success,k) = form_closure(contacts,normal)

    print('\nOutput:')
    print('success: \n%s' % success)
    print('k: \n%s' % k)
