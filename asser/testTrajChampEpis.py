from matplotlib.pyplot import *
from pylab import *
import math
import sys
import re

class point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

def coeffTraj(cl):	#_x5
    [x0, x1, xi, dx0, dx1, q0, q1] = cl
    a = 0.0
    b = 10.0*q1 + 120.0*x1 - 60.0*dx1 - 10.0*q0 - 60.0*dx0 - 120.0*x0
    c = 84.0*dx1 + 96.0*dx0 + 18.0*q0 + 180.0*x0 - 12.0*q1 - 180.0*x1
    d = 3.0*q1 + 60.0*x1 - 24.0*dx1 - 9.0*q0 - 36.0*dx0 - 60.0*x0
    e = q0
    f = dx0
    g = x0
    C = [a, b, c, d, e, f, g]
    return(C)

def coeffTraj_x6(cl): #
    [x0, x1, xi, dx0, dx1, q0, q1] = cl
    a = -1920*xi - 300*dx1 + 30*q0 + 30*q1 + 300*dx0 + 960*x0 + 960*x1
    b = -2040*x0 - 1800*x1 - 660*dx0 - 70*q0 - 50*q1 + 540*dx1 + 3840*xi
    c = -2304*xi - 276*dx1 + 24*q1 + 54*q0 + 456*dx0 + 972*x1 + 1332*x0
    d = -252*x0 - 132*x1 - 96*dx0 - 15*q0 - 3*q1 + 36*dx1 + 384*xi
    e = q0
    f = dx0
    g = x0
    C = [a, b, c, d, e, f, g]
    return(C)
    
def coeffTraj_dq10(cl):  #
    [x0, x1, xi, dx0, dx1, q0, q1] = cl
    a = -300*x1 - 45*q1 + 15*q0 + 120*dx0 + 180*dx1 + 300*x0
    b = -720*x0 - 420*dx1 - 300*dx0 - 40*q0 + 100*q1 + 720*x1
    c = -540*x1 - 66*q1 + 36*q0 + 240*dx0 + 300*dx1 + 540*x0
    d = -120*x0 - 60*dx0 - 60*dx1 - 12*q0 + 12*q1 + 120*x1
    e = q0
    f = dx0
    g = x0
    C = [a, b, c, d, e, f, g]
    return(C)

def coeffDelta(cl):
    [a, b, c, d, e, f, g] = coeffTraj(cl)
    C = [a/5.0, b/4.0, c/3.0, d/2, e/2.0, f, g]
    return(C)

def coeffPosition(cl):
    [a, b, c, d, e, f, g] = coeffTraj(cl)
    C = [a/30.0, b/20.0, c/12.0, d/6.0, e/2.0, f, g]
    return(C)

def conditionsLimites(p0, p1, th0, th1, B):
    dist = math.sqrt(pow(p1.x-p0.x,2) + pow(p1.y - p0.y,2))
    delta0 = point(dist*math.cos(th0), dist*math.sin(th0))
    delta1 = point(dist*math.cos(th1), dist*math.sin(th1))

    #calcul du point intermediaire
    thetaP = math.atan2(p1.y-p0.y, p1.x-p0.x)
    if (th0 > thetaP):
        theta10 = th0 - thetaP
    else:
        theta10 = thetaP - th0
        
    if (th1 > thetaP):
        theta11 = th1 - thetaP
    else:
        theta11 = thetaP - th1
            
    d = dist*(math.sin(theta11)/math.sin(math.pi-theta10-theta11))
    pi = point(p0.x + d*math.cos(th0), p0.y + d*math.sin(th0))

    pm = point((p0.x+p1.x)/2, (p0.y+p1.y)/2)
    pc = point(pm.x + B*(pi.x-pm.x), pm.y + B*(pi.y-pm.y))
    #~ pc = point((p0.x+p1.x+pi.x)/3.0, (p0.y+p1.y+pi.y)/3.0)

    l_condLim = []
    l_condLim.append([p0.x, p1.x, pc.x, delta0.x, delta1.x, 0.0, 0.0])
    l_condLim.append([p0.y, p1.y, pc.y, delta0.y, delta1.y, 0.0, 0.0])
    l_condLim.append([pm.x, pc.x, pi.x])
    l_condLim.append([pm.y, pc.y, pi.y])
    
    return(l_condLim)

def trajectoire(p0, p1, th0, th1, B):
    condLim = conditionsLimites(p0, p1, th0, th1, B)
    l_cX = coeffPosition(condLim[0])
    l_cY = coeffPosition(condLim[1])
    traj = [[], []]
    for v in range(21):
        t = v/20.0
        x = l_cX[0] * pow(t,6) + l_cX[1] * pow(t,5) + l_cX[2] * pow(t,4) + l_cX[3] * pow(t,3) + l_cX[4] * pow(t,2) + l_cX[5] * t + l_cX[6]
        y = l_cY[0] * pow(t,6) + l_cY[1] * pow(t,5) + l_cY[2] * pow(t,4) + l_cY[3] * pow(t,3) + l_cY[4] * pow(t,2) + l_cY[5] * t + l_cY[6]
        traj[0].append(x)
        traj[1].append(y)
        
    traj.append(condLim[2])
    traj.append(condLim[3])

    return traj

def q(p0, p1, th0, th1, B):
    condLim = conditionsLimites(p0, p1, th0, th1, B)
    N = 30
    [a, b, c, d, e, f, g] = coeffTraj(condLim[0])
    print([a, b, c, d, e, f, g])
    l_qx = [a*pow(t/(N*1.0),4) + b*pow(t/(N*1.0),3) + c*pow(t/(N*1.0),2) + d*t/(N*1.0) + e for t in range(N+1)]
    [a, b, c, d, e, f, g] = coeffTraj(condLim[1])
    print([a, b, c, d, e, f, g])
    l_qy = [a*pow(t/(N*1.0),4) + b*pow(t/(N*1.0),3) + c*pow(t/(N*1.0),2) + d*t/(N*1.0) + e for t in range(N+1)]
    
    return([l_qx, l_qy])
    
def delta(p0, p1, th0, th1, B):
    condLim = conditionsLimites(p0, p1, th0, th1, B)
    N = 30
    [a, b, c, d, e, f, g] = coeffDelta(condLim[0])
    print([a, b, c, d, e, f, g])
    l_dx = [a*pow(t/(N*1.0),5) + b*pow(t/(N*1.0),4) + c*pow(t/(N*1.0),3) + d*pow(t/(N*1.0),2) + e*t/(N*1.0) + f for t in range(N+1)]
    [a, b, c, d, e, f, g] = coeffDelta(condLim[1])
    print([a, b, c, d, e, f, g])
    l_dy = [a*pow(t/(N*1.0),5) + b*pow(t/(N*1.0),4) + c*pow(t/(N*1.0),3) + d*pow(t/(N*1.0),2) + e*t/(N*1.0) + f for t in range(N+1)]
    
    return([l_dx, l_dy])

def listeDeplacements():
	logDep = open("matchlog.txt", 'r')
	lines = logDep.readlines()
	logDep.close()
	
	dep = []
	for line in lines:
		res = re.search('\"(\w+)\"', line)
		if (res != None):			
			dico = {"mvt":res.groups()[0]}
			res = re.findall('(\w+)\s*=\s*([\d.-]+)', line)
			if (res != None):
				for coupleKV in res:
					dico[coupleKV[0]] = float(coupleKV[1])
				#~ print dico
				dep.append(dico)
	
	#~ print(len(dep))
	return(dep)
	
	
figure(1)
# affichage de l'image du terrain
terrain = imread('map.png')
extent = (-0.019, 3.098, -0.022, 2.515)
im = imshow(terrain, cmap=cm.hot, origin='lower', extent=extent)

#~ plot([0.185, 2.815, 0.25, 2.75], [0.185, 0.185, 1.972, 1.972], '+r')
#axis([0,25,0,25])
v = axis()
limits = []
for val in v:
	limits.append(val)
limits[0] = -0.022
limits[1] = 3.0
limits[2] = -0.022
limits[3] = 2.122
axis(limits)
	
l_dep = listeDeplacements()

flag_couleur=0
cpt_dep = 0
pF = point(2.815, 0.185)
plot([pF.x], [pF.y], 'or')
hold(True)
thetaF = math.atan2(0.672-pF.y, 2.5-pF.x)
print("thetaF: " + str(thetaF))
l_ptSuppr = [2,3,6,7,10,11,14,15,19,20,23,24,27,28,34,35,38,39,42,43,46,47,50,51,54,55,58,59,62,63,66,67]
for dep in l_dep:
    cpt_dep = cpt_dep + 1
    if (l_ptSuppr.count(cpt_dep) == 0):
        if (dep["mvt"] == 'move'):
            #~ print(dep)
            pI = pF
            pF = point(dep["x"]/1000.0, dep["y"]/1000.0)
            if (cpt_dep == 17) or (cpt_dep == 30):
                thetaI = math.pi + thetaF
                thetaF = math.pi + dep['angle']
            else:
                thetaI = thetaF
                thetaF = dep['angle']
            
            #calcul du point intermediaire
            thetaP = math.atan2(pF.y-pI.y, pF.x-pI.x)
            
            theta10 = thetaP - thetaI
            if theta10 < 0:
                theta10 = theta10 + 2*math.pi
            if (theta10 > math.pi):
                theta10 = 2*math.pi - theta10
                
            theta11 = thetaP - thetaF
            if theta11 < 0:
                theta11 = theta11 + 2*math.pi
            if theta11 > math.pi:
                theta11 = 2*math.pi - theta11
                
            print([thetaI*(180/math.pi),thetaF*(180/math.pi),thetaP*(180/math.pi)])
                
            dist = math.sqrt(pow(pF.x-pI.x,2) + pow(pF.y-pI.y,2))
            d = dist*(math.sin(theta11)/math.sin(math.pi-theta10-theta11))
            pinter = point(pI.x + d*math.cos(thetaI), pI.y + d*math.sin(thetaI))
            
            B = 0.1
            p1 = point(pinter.x-B*cos(thetaI), pinter.y-B*sin(thetaI))
            p2 = point(pinter.x+B*cos(thetaF), pinter.y+B*sin(thetaF))
            chemin = trajectoire(pI, p1, thetaI, thetaI, B)
            l_x = chemin[0]
            l_y = chemin[1]
            chemin = trajectoire(p1, p2, thetaI, thetaF, B)
            l_x.extend(chemin[0])
            l_y.extend(chemin[1])
            chemin = trajectoire(p2, pF, thetaF, thetaF, B)
            l_x.extend(chemin[0])
            l_y.extend(chemin[1])
            if (flag_couleur == 1):
                plot(l_x, l_y, '-r', linewidth=2)
                flag_couleur = 0
            else:
                plot(l_x, l_y, '-b', linewidth=2)
                flag_couleur = 1
            
            plot([pinter.x], [pinter.y], 'oc')
				
        else:
            thetaI = thetaF
            thetaF = math.atan2(dep["y"]/1000.0 - pF.y, dep["x"]/1000.0 - pF.x)
		
        print([cpt_dep, dep])
	
plot([pF.x], [pF.y], 'og')
	
show()
		
sys.exit(2)


pI = point(2.815, 0.185)
thetaI = math.atan2(0.672-pI.y, 2.5-pI.x)
#~ thetaI = 120*(math.pi/180)
pF = point(2.850, 0.722)
pF1 = pF
thetaF = math.atan2(pF.y-0.672, pF.x-2.5)
pF = point(pF.x - 0.05*cos(thetaF), pF.y - 0.05*sin(thetaF))
#~ thetaF = 10*(math.pi/180)
#~ pinter = point(0.46,  0.452)
B = 0.5
#~ chemin = trajectoire(pI, pF, thetaI, thetaF, B)
#~ log_q = q(pI, pF, thetaI, thetaF, B)
#~ log_delta = delta(pI, pF, thetaI, thetaF, B)
chemin = trajectoire(pF, pF1, thetaF, thetaF, B)
log_q = q(pF, pF1, thetaF, thetaF, B)
log_delta = delta(pF, pF1, thetaF, thetaF, B)

#~ sys.exit(2)

figure(1)
# affichage de l'image du terrain
terrain = imread('map.png')
extent = (-0.019, 3.098, -0.022, 2.515)
im = imshow(terrain, cmap=cm.hot, origin='lower', extent=extent)

plot([0.185, 2.815, 0.25, 2.75], [0.185, 0.185, 1.972, 1.972], '+r')
#axis([0,25,0,25])
v = axis()
limits = []
for val in v:
	limits.append(val)
limits[0] = -0.022
limits[1] = 3.0
limits[2] = -0.022
limits[3] = 2.122
axis(limits)
	
# trace des trajectoires
#~ print(chemin)
plot(chemin[0], chemin[1], '-r', linewidth=2)
#~ plot([chemin[0][0], chemin[0][-1]], [chemin[1][0], chemin[1][-1]],'-or')
#~ plot([chemin[0][0], chemin[0][0]+1*cos(thetaI)], [chemin[1][0], chemin[1][0]+1*sin(thetaI)], '-og')
#~ plot([chemin[0][-1], chemin[0][-1]+1*cos(thetaF+math.pi)], [chemin[1][-1], chemin[1][-1]+1*sin(thetaF+math.pi)], '-og')
hold(True)

#~ plot(chemin[2][:2], chemin[3][:2], '-or')
#~ plot(chemin[2][1:], chemin[3][1:], '-ob')
#~ plot([2.5], [0.672], 'ok')

figure(2)
subplot(2,1,1)
plot(range(len(log_q[0])), log_q[0], label = 'qx')
hold(True)
plot(range(len(log_q[1])), log_q[1], label = 'qy')
legend()
grid()
	
subplot(2,1,2)
plot(range(len(log_delta[0])), log_delta[0], label = 'dx')
hold(True)
plot(range(len(log_delta[1])), log_delta[1], label = 'dy')
legend()
grid()
    
show()
