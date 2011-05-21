from sympy import *

a = Symbol('a')
b = Symbol('b')
c = Symbol('c')
d = Symbol('d')
e = Symbol('e')
f = Symbol('f')

q0 = Symbol('q0')
q1 = Symbol('q1')
dx0 = Symbol('dx0')
dx1 = Symbol('dx1')
x0 = Symbol('x0')
x1 = Symbol('x1')

sol = solve([d - q0
		, a + b + c + d - q1
		, e - dx0
		, a/4.0 + b/3.0 + c/2.0 + d + e - dx1
		, f - x0
		, a/20.0 + b/12.0 + c/6.0 + d/2.0 + e + f - x1]
		, [a, b, c, d, e, f]
		)
print(sol)
