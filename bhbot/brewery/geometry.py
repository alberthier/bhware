#!/usr/bin/env python
# encoding: utf-8

import math

import tools




class Segment(object):

    def __init__(self, x1, y1, x2, y2):
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2

        if tools.quasi_equal(self.x1, self.x2):
            self.a = None
            self.b = None
        else:
            self.a = (self.y2 - self.y1) / (self.x2 - self.x1)
            self.b = self.y1 - self.a * self.x1


    def contains(self, x, y):
        if self.a == None:
            # Vertical segment.
            ok = tools.quasi_equal(x, self.x1) and tools.is_between(self.y1, self.y2, y)
        else:
            # General case
            #  Check that the point is on the line
            ok = tools.quasi_equal(self.a * x + self.b, y)
            #  Check that the point is in the segment bounds
            ok = ok and tools.is_between(self.x1, self.x2, x)
            ok = ok and tools.is_between(self.y1, self.y2, y)
        return ok


    def intersects(self, other):
        if self.a is None and other.a is None:
            # Two vertical lines
            return tools.quasi_equal(self.x1, other.x1) and \
                    (tools.is_between(self.y1, self.y2, other.y1)  or \
                     tools.is_between(self.y1, self.y2, other.y2)  or \
                     tools.is_between(other.y1, other.y2, self.y1) or \
                     tools.is_between(other.y1, other.y2, self.y2))
        if self.a is None:
            cross_x = self.x1
            cross_y = other.a * cross_x + other.b
        elif other.a is None:
            cross_x = other.x1
            cross_y = self.a * cross_x + self.b
        elif not tools.quasi_equal(self.a, other.a):
            cross_x = (other.b - self.b) / (self.a - other.a)
            cross_y = self.a * cross_x + self.b
        else:
            # Two segments on the same line
            return tools.quasi_equal(self.b, other.b) and \
                    (tools.is_between(self.y1, self.y2, other.y1)  or \
                     tools.is_between(self.y1, self.y2, other.y2)  or \
                     tools.is_between(other.y1, other.y2, self.y1) or \
                     tools.is_between(other.y1, other.y2, self.y2))

        return self.contains(cross_x, cross_y) and other.contains(cross_x, cross_y)


    def __repr__(self):
        return "Segment({}, {}, {}, {})".format(self.x1, self.y1, self.x2, self.y2)




class Rectangle(object):

    def __init__(self, x, y, x_size, y_size):
        self.x1 = x
        self.y1 = y
        self.x2 = x + x_size
        self.y2 = y + y_size
        self.segments = []
        self.segments.append(Segment(x         , y         , x + x_size, y         ))
        self.segments.append(Segment(x + x_size, y         , x + x_size, y + y_size))
        self.segments.append(Segment(x + x_size, y + y_size, x         , y + y_size))
        self.segments.append(Segment(x         , y + y_size, x         , y         ))


    def contains(self, x, y):
        return tools.is_between(self.x1, self.x2, x) and tools.is_between(self.y1, self.y2, y)


    def intersects(self, segment):
        if self.contains(segment.x1, segment.y1) or self.contains(segment.x2, segment.y2):
            return True
        for shape_segment in self.segments:
            if shape_segment.intersects(segment):
                return True
        return False


    def __repr__(self):
        return "Rectangle({}, {}, {}, {})".format(self.x1, self.y1, self.x2, self.y2)




class RegularPolygon(object):

    def __init__(self, x, y, radius, n_sides):
        self.segments = []
        self.x = x
        self.y = y
        self.radius = radius
        px = x + radius
        py = y
        part = 2.0 * math.pi / float(n_sides)
        for i in range(1, n_sides + 1):
            a = float(i) * part
            nx = x + math.cos(a) * radius
            ny = y + math.sin(a) * radius
            self.segments.append(Segment(px, py, nx, ny))
            px = nx
            py = ny


    def contains(self, x, y):
        segment_to_center = Segment(self.x, self.y, x, y)
        for shape_segment in self.segments:
            if shape_segment.intersects(segment_to_center):
                return False
        return True


    def intersects(self, segment):
        for shape_segment in self.segments:
            if shape_segment.intersects(segment):
                return True
        if self.contains(segment.x1, segment.y1) and self.contains(segment.x2, segment.y2):
            return True
        return False




class Circle(object):

    def __init__(self, x, y, radius):
        self.x = x
        self.x_squared = x ** 2
        self.y = y
        self.y_squared = y ** 2
        self.radius = radius
        self.radius_squared = radius ** 2


    def contains(self, x, y):
        return tools.is_between(0.0, self.radius, tools.distance(self.x, self.y, x, y))


    def intersects(self, segment):
        if self.contains(segment.x1, segment.y1) and self.contains(segment.x2, segment.y2):
            return True
        dx = segment.x2 - segment.x1
        dy = segment.y2 - segment.y1
        a = dx ** 2.0 + dy ** 2.0
        b = 2.0 * (dx * (segment.x1 - self.x) + dy * (segment.y1 - self.y))
        c = segment.x1 ** 2 + segment.y1 ** 2 + self.x_squared + self.y_squared - 2.0 * (segment.x1 * self.x + segment.y1 * self.y) - self.radius_squared
        d = b ** 2 - 4.0 * a * c

        if d < 0.0:
            return False

        u = ((self.x - segment.x1) * dx + (self.y - segment.y1) * dy) / a

        return 0.0 <= u and u <= 1.0
