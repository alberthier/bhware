from sympy import *

t = Symbol('t')
a = Symbol('a')
b = Symbol('b')
c = Symbol('c')

integ = integrate(cos(a*t**2 + b*t +c), t)
print(integ)
