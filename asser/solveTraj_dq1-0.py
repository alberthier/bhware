from sympy import *

a = Symbol('a')
b = Symbol('b')
c = Symbol('c')
d = Symbol('d')
e = Symbol('e')
f = Symbol('f')
g = Symbol('g')

q0 = Symbol('q0')
q1 = Symbol('q1')
dx0 = Symbol('dx0')
dx1 = Symbol('dx1')
x0 = Symbol('x0')
x1 = Symbol('x1')
dq1 = Symbol('dq1')

sol = solve([e - q0
		, a + b + c + d + e -q1
		, f - dx0
		, a/5 + b/4 + c/3 + d/2 + e + f - dx1
		, g - x0
		, a/30 + b/20 + c/12 + d/6 + e/2 + f + g - x1
		, 4*a +3*b + 2*c + d]
		, [a, b, c, d, e, f, g]
		)
print(sol)
