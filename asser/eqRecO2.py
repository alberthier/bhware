from matplotlib.pyplot import *
from math import *
import sys

#liste =  range(1)
#for n in liste:
    #print('a')
#sys.exit(2)

Te = 0.02
T1 = 0.033
T2 = 0.203
K = 0.9#/50.0

def eqRec(l_e, l_s, Te, T1, T2, K):
    cexp1 = exp(-Te/T1)
    cexp2 = exp(-Te/T2)
    res = l_s[-1] * (cexp1 + cexp2) - l_s[-2] * cexp1 * cexp2
    res = res + K * l_e[-2] * (1 - cexp1) * (1 - cexp2)
    return res

l_entree = [0.0, 0.0]
l_sortie = [0.0, 0.0]
N = 100

for n in range(N):
    sNew = eqRec(l_entree, l_sortie)
    l_sortie.append(sNew)
    l_entree.append(1.0)
    
figure(1)
plot(range(N), l_entree[-(N):])
hold(True)
plot(range(N), l_sortie[-(N):])
grid(True)

show()