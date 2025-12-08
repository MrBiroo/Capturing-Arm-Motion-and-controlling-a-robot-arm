import sympy as sp
from sympy import Matrix

#in order to handle invers kinematics
# Define symbolic variables (c1, c2, ..., c6) and (s1, s2, ..., s6)
c1, c2, c3, c4, c5, c6 = sp.symbols('c1 c2 c3 c4 c5 c6')
s1, s2, s3, s4, s5, s6 = sp.symbols('s1 s2 s3 s4 s5 s6')
cp,sp_sym,cth,sth,cfi,sfi = sp.symbols('cp sp cth sth cfi sfi') # Avoid shadowing the sp module

# Define expressions for tc1 and tc2
tc1 = 71 * c2
tc2 = 71 * s2

# Define two symbolic 4x4 matrices t1 and t2
t1 = sp.Matrix([[c1, 0, s1, 0],
                [s1, 0, -c1, 0],
                [0, 1, 0, 0],
                [0, 0, 0, 1]])

t2 = sp.Matrix([[-s2, 0, c2, 0],
                [c2, 0, s2, 0],
                [0, 1, 0, 0],
                [0, 0, 0, 1]])

t3 = sp.Matrix([[-c3, 0, s3, 0],
                [-s3, 0, -c3, 0],
                [0, -1, 0, 1],
                [0, 0, 0, 1]])

# Define the RPY metrix

# Perform matrix multiplication
C = t1 * t2
C2 = t1 * t2 * t3

print(C2)
