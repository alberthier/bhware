#!/usr/bin/env python
# encoding: utf-8

import unittest



class PacketsTestCase(unittest.TestCase):

    def setUp(self):
        self.i = 1

    def test_is_one(self):
        self.assertTrue(self.i == 1)

    def test_two(self):
        self.assertEqual(self.i + 1, 2)

