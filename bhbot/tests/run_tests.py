#!/usr/bin/env python3
# encoding: utf-8

import unittest
import os
import sys




def patch_pythonpath():
    brewery_root_path = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
    to_append = os.path.join(brewery_root_path, "brewery")
    if not to_append in sys.path :
        sys.path.append(to_append)




if __name__ == "__main__":
    patch_pythonpath()
    import definitions
    definitions.setup_definitions()
    names = []
    for f in os.listdir(os.path.dirname(os.path.realpath(__file__))):
        if f.endswith(".py") and f.startswith("test"):
            names.append(f[:-3])
    suite = unittest.defaultTestLoader.loadTestsFromNames(names)

    unittest.TextTestRunner().run(suite)
