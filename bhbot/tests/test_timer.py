# encoding: utf-8

import unittest
import run_tests
run_tests.patch_pythonpath()


import eventloop

from definitions import *


########################################################################


class TestTimer(unittest.TestCase):

    def test_timer_sequence(self):
        self.timers = []
        t1 = eventloop.Timer(self, 1000, None)
        t1.start()
        self.assertEqual(len(self.timers), 1)
        self.assertNotEqual(t1.timeout_date, None)

        t2 = eventloop.Timer(self, 1000, None)
        t2.start()
        self.assertEqual(len(self.timers), 2)
        self.assertNotEqual(t2.timeout_date, None)

        t3 = eventloop.Timer(self, 2000, None)
        t3.start()
        self.assertEqual(len(self.timers), 3)
        self.assertNotEqual(t3.timeout_date, None)

        t4 = eventloop.Timer(self, 3000, None)
        t4.start()
        self.assertEqual(len(self.timers), 4)
        self.assertNotEqual(t4.timeout_date, None)

        t5 = eventloop.Timer(self, 4000, None)
        t5.start()
        self.assertEqual(len(self.timers), 5)
        self.assertNotEqual(t5.timeout_date, None)


        self.check_timers_order()
        t2.stop()
        self.assertEqual(len(self.timers), 4)

        self.check_timers_order()
        t4.stop()
        self.assertEqual(len(self.timers), 3)

        self.check_timers_order()
        t5.stop()
        self.assertEqual(len(self.timers), 2)

        self.check_timers_order()
        t3.stop()
        self.assertEqual(len(self.timers), 1)

        self.check_timers_order()
        t1.stop()
        self.assertEqual(len(self.timers), 0)


    def check_timers_order(self):
        for i in range(len(self.timers) - 1):
            t1 = self.timers[i]
            t2 = self.timers[i + 1]
            self.assertTrue(t1 <= t2)
