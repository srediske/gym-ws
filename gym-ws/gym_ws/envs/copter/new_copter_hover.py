# import pdb # python debugger
import gym
import rospy
import roslaunch
import time
import numpy as np

import os
import subprocess
import math

from gym import utils, spaces
from gym_ws.envs import gazebo_env
from gym.utils import seeding

from mavros_msgs.msg import OverrideRCIn
from sensor_msgs.msg import LaserScan, NavSatFix
from std_msgs.msg import Float64
from gazebo_msgs.msg import ModelStates

from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from std_srvs.srv import Empty


# pdb.set_trace()
class new_CopterEnv(gazebo_env.GazeboEnv):

    def _launch_apm(self):
        sim_vehicle_py = str(
            os.environ["ARDUPILOT_PATH"]) + "/Tools/autotest/sim_vehicle.py"
        subprocess.Popen(["xterm","-e", sim_vehicle_py,"-j4","-v","ArduCopter","-f","gazebo-iris","--console"])

    def _takeoff(self, altitude):
        print("Waiting for mavros...")
        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('/mavros/global_position/rel_alt', Float64, timeout=5)
            except:
                pass

	print("Set mode")
	rospy.wait_for_service('/mavros/set_mode')
	try:
		modeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)
		modeResponse = modeService(0, 'GUIDED_NOGPS')
		rospy.loginfo("\nMode Response: " + str(modeResponse))
		print("Mode set")
	except rospy.ServiceException as e:
		print("Service call failed: %s" %e)

	time.sleep(20)
	print("Set arming")
	rospy.wait_for_service('/mavros/cmd/arming')
	try:
		armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
		armResponse = armService(True)
		rospy.loginfo(armResponse)
		print("Armed")
	except rospy.ServiceException as e:
		print("Service call failed: %s" %e)

        takeoff_successful = False
        while not takeoff_successful:
            print("Taking off...")
            alt = altitude
            err = alt * 0.1  # 10% error

             #pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)

             #msg = OverrideRCIn()
             #msg.channels[0] = 0 # Roll
             #msg.channels[1] = 0 # Pitch
             #msg.channels[2] = 1500 # Throttle
             #msg.channels[3] = 0    # Yaw
             #msg.channels[4] = 0
             #msg.channels[5] = 0
             #msg.channels[6] = 0
             #msg.channels[7] = 0
             #self.pub.publish(msg)

            # Takeoff
            rospy.wait_for_service('mavros/cmd/takeoff')
            try:
                self.takeoff_proxy(0, 0, 0, 0, alt)  # 1m altitude
            except (rospy.ServiceException) as e:
                print ("mavros/cmd/takeoff service call failed: %s" % e)

            # Wait 3 seconds
            time.sleep(3)

            alt_msg = None
            while alt_msg is None:
                try:
                    alt_msg = rospy.wait_for_message('/gazebo/model_states', ModelStates, timeout=10)
                except:
                    pass

            #erlecopter_index = 0
            #print("Finding drone index")
            #for name in alt_msg.name:
            #    if name == "erlecopter":
            iris_zed_index = 0
            print("Finding drone index")
            for name in alt_msg.name:
                if name == "iris_demo":

                    break
                else:
           #         erlecopter_index += 1
           # erlecopter_alt = alt_msg.pose[erlecopter_index].position.z * 2
           # if erlecopter_alt > (alt - err):
                    iris_zed_index += 1
            iris_zed_alt = alt_msg.pose[iris_zed_index].position.z * 2
            if iris_zed_alt > (alt - err):
                takeoff_successful = True
                print("Takeoff successful")
            else:
                print("Takeoff failed, retrying...")

        # Set ALT_HOLD mode
        rospy.wait_for_service('mavros/set_mode')
        try:
            self.mode_proxy(0, 'ALT_HOLD')
        except (rospy.ServiceException) as e:
            print ("mavros/SetMode service call failed: %s" % e)

    def _pause(self, msg):
        programPause = raw_input(str(msg)) # changed 03.07.19 SR and added lines below
        #rospy.wait_for_service('/gazebo/pause_physics')
        #try:
        #    self.pause()
        #except rospy.ServiceException, e:
        #    print ("/gazebo/pause_physics service call failed")

    def __init__(self):

        self._launch_apm()

        RED = '\033[91m'
        BOLD = '\033[1m'
        ENDC = '\033[0m'
        LINE = "%s%s##############################################################################%s" % (
        RED, BOLD, ENDC)
        msg = "\n%s\n" % (LINE)
        msg += "%sLoad Copter parameters in MavProxy console (sim_vehicle.py):%s\n\n" % (BOLD, ENDC)
        msg += "MAV> param load %s\n\n" % (str("~/gym-ws/gym-ws/gym_ws/envs/params/new_3DR_Iris+_AC34.param"))
        msg += "%sThen, press <Enter> to launch Gazebo...%s\n\n%s" % (BOLD, ENDC, LINE)

        # Launch the simulation with the given launchfile name
        gazebo_env.GazeboEnv.__init__(self, "new_Hover-v0.launch")

        self.action_space = spaces.Discrete(4)  # F, L, R, B
        # self.observation_space = spaces.Box(low=0, high=20) #laser values
        self.reward_range = (-np.inf, np.inf)

        self.initial_latitude = None
        self.initial_longitude = None

        self.current_latitude = None
        self.current_longitude = None

        self.diff_latitude = None
        self.diff_longitude = None

        self.max_distance = 1.6

        self.pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)

        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)

        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)

        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_world', Empty) # Resets the model's poses
        #self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_simulation', Empty) # Resets the entire simulation including the time

        self.mode_proxy = rospy.ServiceProxy('mavros/SetMode', SetMode)

        self.arm_proxy = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)

        self.takeoff_proxy = rospy.ServiceProxy('mavros/cmd/takeoff', CommandTOL)

        countdown = 10
        while countdown > 0:
            print ("Taking off in %ds" % countdown)
            countdown -= 1
            time.sleep(1)

        self._takeoff(2)

        self._seed()

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def _state(self, action):
        return discretized_ranges, done

    def step(self, action):
        msg = OverrideRCIn()

        if action == 0:  # FORWARD
            msg.channels[0] = 1500  # Roll
            msg.channels[1] = 1450  # Pitch
        elif action == 1:  # LEFT
            msg.channels[0] = 1450  # Roll
            msg.channels[1] = 1500  # Pitch
        elif action == 2:  # RIGHT
            msg.channels[0] = 1550  # Roll
            msg.channels[1] = 1500  # Pitch
        elif action == 3:  # BACKWARDS
            msg.channels[0] = 1500  # Roll
            msg.channels[1] = 1550  # Pitch

        msg.channels[2] = 1500  # Throttle
        msg.channels[3] = 0  # Yaw
        msg.channels[4] = 0
        msg.channels[5] = 0
        msg.channels[6] = 0
        msg.channels[7] = 0

        self.pub.publish(msg)

        observation = self._get_position()

        dist = self.center_distance()
        done = dist > self.max_distance

        reward = 0
        if done:
            reward = -100
        else:
            reward = 10 - dist * 8

        return observation, reward, done, {}

    def _killall(self, process_name):
        pids = subprocess.check_output(["pidof", process_name]).split()
        for pid in pids:
            os.system("kill -9 " + str(pid))

    def _relaunch_apm(self):
        pids = subprocess.check_output(["pidof", "ArduCopter.elf"]).split()
        for pid in pids:
            os.system("kill -9 " + str(pid))

        grep_cmd = "ps -ef | grep ardupilot"
        result = subprocess.check_output([grep_cmd], shell=True).split()
        pid = result[1]
        os.system("kill -9 " + str(pid))

        grep_cmd = "ps -af | grep sim_vehicle.py"
        result = subprocess.check_output([grep_cmd], shell=True).split()
        pid = result[1]
        os.system("kill -9 " + str(pid))

        self._launch_apm()

    def _to_meters(self, n):
        return n * 100000.0

    def _get_position(self):
        # read position data
        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('/mavros/global_position/global', NavSatFix, timeout=5)
            except:
                pass

        self.current_latitude = self._to_meters(data.latitude)
        self.current_longitude = self._to_meters(data.longitude)

        if self.initial_latitude == None and self.initial_longitude == None:
            self.initial_latitude = self.current_latitude
            self.initial_longitude = self.current_longitude
            print("Initial latitude : %f, Initial Longitude : %f" % (self.initial_latitude, self.initial_longitude,))

        print("Current latitude : %f, Current Longitude : %f" % (self.current_latitude, self.current_longitude,))

        self.diff_latitude = self.current_latitude - self.initial_latitude
        self.diff_longitude = self.current_longitude - self.initial_longitude

        print("Diff latitude: %f, Diff Longitude: %f" % (self.diff_latitude, self.diff_longitude,))

        return self.diff_latitude, self.diff_longitude

    def center_distance(self):
        return math.sqrt(self.diff_latitude ** 2 + self.diff_longitude ** 2)

    def reset(self):
        # Resets the state of the environment and returns an initial observation.
        rospy.wait_for_service('/gazebo/reset_world')  # Resets the model's poses
        # rospy.wait_for_service('/gazebo/reset_simulation') # Resets the entire simulation including the time
        try:
            # reset_proxy.call()
            self.reset_proxy()
        except (rospy.ServiceException) as e:
            print ("/gazebo/reset_world service call failed")

        # Relaunch autopilot
        self._relaunch_apm()

        self._takeoff(2)

        self.initial_latitude = None
        self.initial_longitude = None

        return self._get_position()
