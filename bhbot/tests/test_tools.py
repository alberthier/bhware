# encoding: utf-8

import unittest
import run_tests
run_tests.patch_pythonpath()


import tools


########################################################################


class Tests(unittest.TestCase):

    def test_quasi_equal(self):
        self.assertTrue(tools.quasi_equal(1.00000001, 1.0))
        self.assertFalse(tools.quasi_equal(1.00000001, 4.0))
        self.assertTrue(tools.quasi_equal(tools.EPSILON, tools.EPSILON + tools.EPSILON / 2.0))
        self.assertFalse(tools.quasi_equal(tools.EPSILON, tools.EPSILON * 2.0))


    def test_quasi_null(self):
        self.assertTrue(tools.quasi_null(0.0))
        self.assertTrue(tools.quasi_null(0.0000001))
        self.assertFalse(tools.quasi_null(1.0))
        self.assertTrue(tools.quasi_null(tools.EPSILON / 2.0))
        self.assertFalse(tools.quasi_null(tools.EPSILON))

