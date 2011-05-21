from numpy import matrix 
from numpy import linalg 

#~ print A.T                                    # Transpose of A. 
#~ print A*x                                    # Matrix multiplication of A and x. 
#~ print A.I                                    # Inverse of A. 
#~ print linalg.solve(A, x)     # Solve the linear equation system. 

A = matrix( [[1,1,1,1],[12,15,20,30],[2,3,5,10],[1,3,10,40]]) # Creates a matrix. 
invA = A.I
print(invA)                                    # Inverse of A.
