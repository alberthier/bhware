#!/usr/bin/env python
# -*- coding: iso8859-1 -*- #

terrain = """  XXXXXXXXXXXXXXXXXXXXXX
               XO   D    O          X
               XO        O          X
               XO        O      OOOOX
               XO        O      OA  X
               XO        O      O   X
               XO        O      O   X
               XO        O      O   X
               XO        O      O   X
               X  OOOOOOOO      O   X
               X                O   X
               X                O   X
               X                O   X
               X                O   X
               X                    X
               XXXXXXXXXXXXXXXXXXXXXX"""               

class Map(object):
   def __init__(self, s=None):
      self.tabTerrain = None
      self.points = {}
      if s is not None :
         self.load_from_string(s)
   
   def load_from_string(self,s):               
      tailleX = 0
      tailleY = 0

      self.tabTerrain=[]

      startPoint = (0,0)
      endPoint = (0,0)

      for l in terrain.splitlines() :
         y = tailleY
         self.tabTerrain.append([])
         l= l.strip()
         tailleX = max(tailleX, len(l))
         for x,elem in enumerate(l) :
            val = 0
            if elem == "X" :
               val = 64
            elif elem == "O" :
               val = 63
            elif elem == "D" :
               startPoint = [x, y]
            elif elem == "A" :
               endPoint = [x, y]
            
            self.tabTerrain[tailleY].append(val)
            
            
         tailleY = tailleY +1
         
      print u"Dimensions du terrain en mémoire", tailleX, tailleY
      print u"Taille maximum en mémoire de la liste des pts", tailleX * tailleY

      return startPoint, endPoint

   def print_(self):
      for line in self.tabTerrain :
         print "".join(map(lambda x : " %2d" % (x), line))

   def get_reachable_neighbours( self, point, fn ):
      """ Reachable neighbours of point which satisfy function fn"""
      x, y = point.to_array()
      ret = []
      for iy in range(y-1, y+2) :
         for ix in range(x-1, x+2) :
            if fn(ix,iy) :
               ret.append([ ix, iy ] )
      ret.remove(point.to_array())
      # print point, "renvoi de ", ret
      return ret

   def clone(self):
      ret = Map()
      ret.tabTerrain = self.tabTerrain[:]
      ret.points = self.points.copy()
      return ret

   def get_point(self, coord):
      return self.points.setdefault(coord[0],{}).setdefault(coord[1],Point(*coord))

   def is_reachable(self, coord):
      x,y=coord
      return self.tabTerrain[y][x] == 0


class Point(object):
   def __init__(self, x, y):
      self.x = x
      self.y = y
      self.g = 0
      self.h = 0
   def __repr__(self):
      return "Point(%d,%d)" % (self.x, self.y)
   def get_f(self):
      f = self.g+self.h
      # print repr(self), "get_f", f
      return f 
   def to_array(self):
      return [self.x, self.y]
   def set_g(self, g):
      # print repr(self), "set_g", g
      self.g = g
   def set_h(self, h):
      self.h = h


   

def dist_manhattan(p1, p2):
   return abs(p1.x - p2.x) + abs(p1.y - p2.y )

def astar(startPoint, endPoint, terrain):
   start = terrain.get_point(startPoint)
   end = terrain.get_point(endPoint)

   openList = [start]
   parents = {}
   closedList = []

   print "debut : ", start
   print "fin :", end

   # print openList

   #statistiques

   stats={ "openListMax" : 0, "closedListMax" : 0, "gMax" : 0, "hMax" : 0  }


   iteration = 0

   while len(openList) > 0 and not end in closedList:
      iteration+=1
      top = openList[0]
      openList.remove(top)
      neighbours = terrain.get_reachable_neighbours(top,lambda x,y : terrain.is_reachable([x,y]))
      # a_rajouter = [ n for n in neighbours if n not in closedList ]
      ptTop = top
      print "top", top, top.get_f(), top.g, top.h
      for n in neighbours :
         ptNew = terrain.get_point(n)
         if ptNew in closedList :
            continue
         newG = ptTop.g + 1
         if ptNew not in openList or newG < ptNew.g :
            print "trouve meilleur que", ptNew.g, ">", newG,"pour",ptNew
            parents[ptNew] = ptTop
            ptNew.set_g(newG)
            newH = dist_manhattan(ptNew, start)
            ptNew.set_h(newH)
            stats["gMax"]=max(stats["gMax"], newG)
            stats["hMax"]=max(stats["hMax"], newH)   
            openList.append(ptNew)
      
      closedList.append(top)
      openList.sort(cmp = lambda x, y : cmp(x.get_f(), y.get_f()))
      
      stats["openListMax"]=max(stats["openListMax"], len(openList))
      stats["closedListMax"]=max(stats["closedListMax"], len(closedList))   
      # print [ (n, n.get_f()) for n in openList]
      # print "iteration", iteration

   chemin = []
   nextP = terrain.get_point( endPoint )
   chemin.append(nextP.to_array())

   while not startPoint in chemin :
      parent = parents[nextP]
      chemin.append(parent.to_array())
      nextP = parent


   print "iterations :", iteration   

   print "chemin :", chemin
   print "longueur :", len(chemin)
      
   print openList

   disp_terrain = terrain.clone()

   for y in range(len(disp_terrain.tabTerrain)) :
      l = disp_terrain.tabTerrain[y]
      for x in  range(len(l)) :
         if [x, y] in chemin :
            disp_terrain.tabTerrain[y][x] = 1

   for line in disp_terrain.tabTerrain :
      print "".join(map(lambda x : " %2d" % (x), line))
      #l = [ " %2d" % (x) if x > 0 else " . " ] 
      #print "".join( l )      

   for y in range(len(disp_terrain.tabTerrain)) :
      l = disp_terrain.tabTerrain[y]
      for x in  range(len(l)) :
         disp_terrain.tabTerrain[y][x] = disp_terrain.get_point([x,y]).h

   for line in disp_terrain.tabTerrain :
      print "".join(map(lambda x : " %2d" % (x), line))

   print "closed list",len(closedList)

   # import pdb
   # 
   # pdb.set_trace()

   # print parents
      
   print "Statistiques :", stats   

   # print closedList

if __name__ == "__main__" :
   t = Map()

   startPoint, endPoint = t.load_from_string(terrain)
   astar(startPoint, endPoint, t)
