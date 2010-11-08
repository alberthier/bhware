#!/usr/bin/env python
# encoding: utf-8

import unittest
import os
import sys




def update_pythonpath():
    brewery_root_path = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
    sys.path.append(os.path.join(brewery_root_path, "brewery"))




if __name__ == "__main__":
    update_pythonpath()
    names = []
    for f in os.listdir(os.path.dirname(os.path.realpath(__file__))):
        if f.endswith(".py") and f.startswith("test"):
            names.append(f[:-3])
    suite = unittest.defaultTestLoader.loadTestsFromNames(names)

    unittest.TextTestRunner().run(suite)
