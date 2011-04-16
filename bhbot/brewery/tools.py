# various tools

import math
import logger

epsilon = 1e-6

def quasi_equal(f1,f2) :
	return abs(f1-f2)<epsilon

def quasi_null(f1) :
	# ret = -epsilon<f1<epsilon
	# logger.log("quasi_null {0}=={1}".format(f1,ret))
	return -epsilon<f1<epsilon

import unittest

class Tests(unittest.TestCase):
	def test_quasi_equal(self):
		self.assertTrue(quasi_equal(1.00000001,1.0))
		self.assertFalse(quasi_equal(1.00000001,4.0))

	def test_quasi_null(self):
		self.assertTrue(quasi_null(0.0))
		self.assertTrue(quasi_null(0.0000001))


if __name__ == "__main__":
	unittest.main()