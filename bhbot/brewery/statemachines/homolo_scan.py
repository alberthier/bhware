import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__)))

from default_simple import *

scan_trajectory = [ { "pos" : list(Cell(0,4).down_right()), "dir" : DIRECTION_BACKWARD }
                  # , { "pos" : list(Cell(0,4).down_middle()), "angle" : ANGLE_W }
                  , { "angle" : ANGLE_W }
                  , { "pos" : list(Cell(0,4).down_middle()) }
                  , { "angle" : ANGLE_N }
				  , { "pos" : list(Cell(0,0).down_middle()) }
                  ]

homologation_trajectory.extend(scan_trajectory)

# class HomologationEnd(statemachine.State) :
#     """End of homologation"""
#     def __init__(self):
#         statemachine.State.__init__(self)
    
#     def on_enter(self):
#         logging.log("Homologation ended")


            

