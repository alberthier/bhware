from matplotlib.pyplot import *
from pylab import *
import math
import sys
import re

class point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

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
