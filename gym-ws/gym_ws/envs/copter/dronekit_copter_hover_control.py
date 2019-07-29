#!/usr/bin/env python

from dronekit_copter_hover import *

# Launch APM SITL
dronekit_CopterEnv._launch_apm()

RED = '\033[91m'
BOLD = '\033[1m'
ENDC = '\033[0m'
LINE = "%s%s##############################################################################%s" % (
RED, BOLD, ENDC)
msg = "\n%s\n" % (LINE)
msg += "%sLoad Copter parameters in MavProxy console (sim_vehicle.py):%s\n\n" % (BOLD, ENDC)
msg += "MAV> param load %s\n\n" % (str("~/gym-ws/gym-ws/gym_ws/envs/params/new_3DR_Iris+_AC34.param"))
msg += "%sThen, press <Enter> to launch Gazebo...%s\n\n%s" % (BOLD, ENDC, LINE)

# Wait until SITL has finished loading, then confirm with enter
dronekit_CopterEnv._pause(msg)

# Launch the simulation with the given launchfile and connect dronekit API with SITL
dronekit_CopterEnv.__init__()



