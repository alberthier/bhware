#!/usr/bin/env python
# encoding: utf-8

terrain_width = 3000
terrain_heigth = 2100
resolution = 50
size_x = terrain_width / resolution
size_y = terrain_heigth / resolution

def manhattan_distance( (x1,y1), (x2,y2) ) :
	return abs ( x1 - x2 ) + (y1-y2)

class Cell(object) :
	def __init__(self, x, y):
		self.x = x
		self.y = y
		self.real_x = x * resolution
		self.real_y = y * resolution

class Map(object) :
	""" cells are stored in a map"""
	def __init():
		self.cells = {}
	
	def initmap() :
		for y in xrange(size_y) :
			for x in xrange(size_x) :
				self.cells.setdefault(y,{})[x] = Cell(x,y)

	def __getcell(self,x,y) :
		return self.cells[y][x]


	def get_cells_manhattan_range(self, x, y, range ) :
		base_x = int (x / resolution)
		base_y = int (y / resolution)
		for ix in xrange( base_x - int(range / resolution), base_x + int (range / resolution) ) :
			for iy in yrange( base_y - int(range / resolution), base_y + int (range / resolution) ) :
				yield self.__getcell(ix,iy)

	def get_cells_range(self, x, y, range ) :
		pass
		

m = Map()

initial_pawns = [ (750, 525) ]

def populate() :
	for p in initial_pawns :
		


if __name__ == "__main__" :
	pass
