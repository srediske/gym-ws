#!/usr/bin/env python

#import pdb # python debugger
import gym
import rospy
import time
import numpy as np
import os
import subprocess
import math

from gym import utils, spaces
from gym_ws.envs import gazebo_env
from gym.utils import seeding

import argparse
from dronekit import VehicleMode, LocationGlobalRelative, connect, LocationGlobal
from ordinary import AutoTest, NotAchievedException

from mavros_msgs.msg import OverrideRCIn
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64
from gazebo_msgs.msg import ModelStates
from pymavlink import mavutil # Needed for command message definitions

from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from std_srvs.srv import Empty

# pdb.set_trace()
class dronekit_CopterEnv(gazebo_env.GazeboEnv, AutoTest):

    # Function to connect DroneKit to SITL
#    def _connect(self):
#        parser = argparse.ArgumentParser()
#        parser.add_argument('--connect', default='127.0.0.1:14550') # Set startup argument --connect with SITL-Output on UDP
#        args = parser.parse_args()

#        # Connect to the Drone
#        self.progress("Connect DroneKit to Drone on: %s" % args.connect)
#        vehicle = connect(args.connect, baud=57600, wait_ready=True)
#        self.progress("DroneKit connected")
#        return vehicle

    # Function to launch SITL resp. sim_vehicle.py
    def _launch_apm(self):
        sim_vehicle_py = str(
            os.environ["ARDUPILOT_PATH"]) + "/Tools/autotest/sim_vehicle.py"
        subprocess.Popen(["xterm","-e", sim_vehicle_py,"-j4","-v","ArduCopter","-f","gazebo-iris","--console"])
        #subprocess.Popen([sim_vehicle_py,"-j4","-v","ArduCopter","-f","gazebo-iris","--console"])

    def send_attitude_target(self, roll_angle=0.0, pitch_angle=0.0, yaw_angle=None, yaw_rate=0.0, use_yaw_rate=False, thrust=0.5):
        """
        use_yaw_rate: the yaw can be controlled using yaw_angle OR yaw_rate.
                      When one is used, the other is ignored by Ardupilot.
        thrust: 0 <= thrust <= 1, as a fraction of maximum vertical thrust.
                Note that as of Copter 3.5, thrust = 0.5 triggers a special case in
                the code for maintaining current altitude.
        """
        #vehicle = self._connect()
        if yaw_angle is None:
            # this value may be unused by the vehicle, depending on use_yaw_rate
            yaw_angle = self.vehicle.attitude.yaw
        # Thrust >  0.5: Ascend
        # Thrust == 0.5: Hold the altitude
        # Thrust <  0.5: Descend
        msg = self.vehicle.message_factory.set_attitude_target_encode(
            0,  # time_boot_ms
            1,  # Target system
            1,  # Target component
            0b00000000 if use_yaw_rate else 0b00000100,
            self.to_quaternion(roll_angle, pitch_angle, yaw_angle),  # Quaternion
            0,  # Body roll rate in radian
            0,  # Body pitch rate in radian
            math.radians(yaw_rate),  # Body yaw rate in radian/second
            thrust  # Thrust
        )
        self.vehicle.send_mavlink(msg)

    def set_attitude(self,roll_angle=0.0, pitch_angle=0.0, yaw_angle=None, yaw_rate=0.0, use_yaw_rate=False, thrust=0.5, duration=0):
        """
        Note that from AC3.3 the message should be re-sent more often than every
        second, as an ATTITUDE_TARGET order has a timeout of 1s.
        In AC3.2.1 and earlier the specified attitude persists until it is canceled.
        The code below should work on either version.
        Sending the message multiple times is the recommended way.
        """
        self.send_attitude_target(roll_angle, pitch_angle,
                             yaw_angle, yaw_rate, False,
                             thrust)
        start = time.time()
        while time.time() - start < duration:
            self.send_attitude_target(roll_angle, pitch_angle,
                                 yaw_angle, yaw_rate, False,
                                 thrust)
            time.sleep(0.1)
        # Reset attitude, or it will persist for 1s more due to the timeout
        self.send_attitude_target(0, 0,
                             0, 0, True,
                             thrust)

    def to_quaternion(self,roll=0.0, pitch=0.0, yaw=0.0):
        """
        Convert degrees to quaternions
        """
        t0 = math.cos(math.radians(yaw * 0.5))
        t1 = math.sin(math.radians(yaw * 0.5))
        t2 = math.cos(math.radians(roll * 0.5))
        t3 = math.sin(math.radians(roll * 0.5))
        t4 = math.cos(math.radians(pitch * 0.5))
        t5 = math.sin(math.radians(pitch * 0.5))

        w = t0 * t2 * t4 + t1 * t3 * t5
        x = t0 * t3 * t4 - t1 * t2 * t5
        y = t0 * t2 * t5 + t1 * t3 * t4
        z = t1 * t2 * t4 - t0 * t3 * t5

        return [w, x, y, z]

    # Takeoff for arming and takeoff to a specified altitude
    def _takeoff(self, altitude):
        #vehicle = self._connect()
        self.progress("Basic pre-arm checks")
        # Don't try to arm until autopilot resp. SITL is ready
        while not self.vehicle.is_armable:
            self.progress("Waiting for Drone to initialise...")
            time.sleep(1)
        self.progress("Arming motors")

    #    vehicle.mode = VehicleMode("GUIDED")
    #    while not vehicle.mode.name=="GUIDED":
    #        time.sleep(1)
    #    self.progress("Changed mode to %s" % vehicle.mode.name)
    #    vehicle.armed = True

        desired_mode = 'STABILIZE'
        self.vehicle.mode = VehicleMode(desired_mode)
        while self.vehicle.mode != desired_mode:
            self.vehicle.mode = VehicleMode(desired_mode)
            time.sleep(1)
        self.progress("Changed mode to %s" % self.vehicle.mode.name)

        while not self.vehicle.armed:
            self.progress("Waiting for arming...")
            self.vehicle.armed = True
            time.sleep(1)
        self.progress("Armed")

        self.progress("Taking off!")
        ##### CONSTANTS #####
        DEFAULT_TAKEOFF_THRUST = 0.7
        SMOOTH_TAKEOFF_THRUST = 0.6
    #    vehicle.simple_takeoff(altitude) # Take off to target altitude for GUIDED mode
        thrust = DEFAULT_TAKEOFF_THRUST
        while True:
            #vehicle.channels.overrides[3] = 2000
            #self.progress("Ch3 override: %s" % vehicle.channels.overrides[3])

            current_altitude = self.vehicle.location.global_relative_frame.alt
            self.progress("Altitude: %f  Desired: %f" % (current_altitude, altitude))
            # Break and return from function just below target altitude.
            if current_altitude >= altitude*0.95:
                #vehicle.channels.overrides[3] = 1500
                self.progress("Reached target altitude")
                #self.progress("Begin hovering")
                #self.progress("Ch3 override: %s" % self.vehicle.channels.overrides[3])
                break
            elif current_altitude >= altitude*0.6:
                thrust = SMOOTH_TAKEOFF_THRUST
            self.set_attitude(thrust = thrust)
            time.sleep(0.2)

        self.progress("DEBUG MARKER")
        #time.sleep(60)

    #    # Check that vehicle has reached takeoff altitude
    #    while True:
    #        #self.progress(" Altitude: %s" % vehicle.location.global_relative_frame.alt)
    #        current_altitude = vehicle.location.global_relative_frame.alt
    #        self.progress("Altitude: %f  Desired: %f" % (current_altitude, altitude))
    #        # Break and return from function just below target altitude.
    #        if vehicle.location.global_relative_frame.alt >= altitude * 0.95:
    #            self.progress("Reached target altitude")
    #            break
    #        time.sleep(10)

        # Change mode to STABILIZE with Thrust on 1500
        # that triggers a special case in the ardupilot code for maintaining current altitude
        # for the simplest possible hover-example, where the RL has only to control Roll & Pitch




    def _pause(self, msg):
        programPause = raw_input(str(msg)) # changed 03.07.19 SR and added lines below
        #rospy.wait_for_service('/gazebo/pause_physics')
        #try:
        #    self.pause()
        #except rospy.ServiceException, e:
        #    print ("/gazebo/pause_physics service call failed")

    def __init__(self):

        self._launch_apm()
        self.mavproxy = None
        self.mav = None

        RED = '\033[91m'
        BOLD = '\033[1m'
        ENDC = '\033[0m'
        LINE = "%s%s##############################################################################%s" % (
        RED, BOLD, ENDC)
        msg = "\n%s\n" % (LINE)
        msg += "%sLoad Copter parameters in MavProxy console (sim_vehicle.py):%s\n\n" % (BOLD, ENDC)
        msg += "MAV> param load %s\n\n" % (str("~/gym-ws/gym-ws/gym_ws/envs/params/new_3DR_Iris+_AC34.param"))
        msg += "%sThen, press <Enter> to launch Gazebo...%s\n\n%s" % (BOLD, ENDC, LINE)
        self._pause(msg)

        # Launch the simulation with the given launchfile name
        gazebo_env.GazeboEnv.__init__(self, "new_Hover-v0.launch")
        time.sleep(25)
        #self._connect()
        #vehicle = self.connect()
        parser = argparse.ArgumentParser()
        parser.add_argument('--connect', default='127.0.0.1:14550') # Set startup argument --connect with SITL-Output on UDP
        args = parser.parse_args()

        # Connect to the Drone
        self.progress("Connect DroneKit to Drone on: %s" % args.connect)
        self.vehicle = connect(args.connect, baud=57600, wait_ready=True)
        self.progress("DroneKit connected")
        time.sleep(1)

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
        self._takeoff(5)
        self.progress("Hold position for 10 seconds")
        self.set_attitude(duration = 10)
        self._seed()

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def _state(self, action):
        return discretized_ranges, done

    def step(self, action):
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/unpause_physics service call failed")

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
        #pids = subprocess.check_output(["pidof", "ArduCopter"]).split()
        #for pid in pids:
        #    os.system("kill -9 " + str(pid))

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

        self._takeoff(5)

        self.initial_latitude = None
        self.initial_longitude = None

        return self._get_position()
