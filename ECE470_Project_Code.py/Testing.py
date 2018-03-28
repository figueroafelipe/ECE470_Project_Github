import ECE470_Project_Functions as myfct
import numpy as np


p = np.matrix([1,2,3])
p.shape = (3,1)

print('\np: ')
print(p, end='\n\n')

p_skew = myfct.skew3(p)

print('\np_skew: ')
print(p_skew, end='\n\n')

V = np.matrix([1,2,3,4,5,6])
V.shape = (6,1)

print('\nV: ')
print(V, end='\n\n')

V_skew = myfct.skew4(V)

print('\nV_skew: ')
print(V_skew, end='\n\n')

T = np.matrix([[1,2,3,10], [4,5,6,11], [7,8,9,12], [0,0,0,1]])

print('\nT: ')
print(T, end='\n\n')

T_adj = myfct.Adj(T)

print('\nT_adj: ')
print(T_adj, end='\n\n')





