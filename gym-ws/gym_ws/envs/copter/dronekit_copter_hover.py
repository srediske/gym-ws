#!/usr/bin/env python

import rospy
import time
import numpy as np
import os
import subprocess
import math
import argparse

from gym import spaces
from gym_ws.envs import gazebo_env
from gym.utils import seeding
from dronekit import VehicleMode, connect, LocationGlobalRelative
from sensor_msgs.msg import NavSatFix
from std_srvs.srv import Empty


class dronekit_CopterEnv(gazebo_env.GazeboEnv):

    @staticmethod
    def progress(text):
        """Display progress text."""
        print("GYM-WS: " + text)

    def __init__(self):
        self._launch_apm()  # Launch SITL
        gazebo_env.GazeboEnv.__init__(self, "new_Hover-v0.launch")  # Launch gazebo with the given launchfile name
        time.sleep(25)  # Wait 25s to load Gazebo and connect SITL before continue
        self.vehicle = self._connect()  # Connect Dronekit with SITL
        time.sleep(2)
        self.action_space = spaces.Discrete(4)  # Set the number of action spaces, in this example 4: Forward, Left, Right, Backwards
        self.reward_range = (-np.inf, np.inf)  # Reward range: -infinite to infinite
        self.initial_latitude = None  # Basically the latitude from the origin point or the startpoint of the Drone
        self.initial_longitude = None  # Basically the longitude from the origin point or the startpoint of the Drone
        self.current_latitude = None  # The measured latitude
        self.current_longitude = None  # The measured longitude
        self.diff_latitude = None  # The latitude difference from Drone to origin
        self.diff_longitude = None  # The longitude difference from Drone to origin
        self.max_distance = 1.6  # Set the max distance of the Drone from the origin (meter) before the simulation reset
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_world', Empty)  # Resets the model's poses
        # self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_simulation', Empty) # Resets the entire simulation including the time,
        # which causes a connection loss between SITL & Gazebo,therefore its much faster to go with 'reset_world'
        self._takeoff(2)  # First takeoff, not really necessary and does not contribute to the RL
        self._seed()

    def _connect(self):
        """
        purpose: Connect Dronekit with APM SITL
        param:
        return:
            vehicle: necessary to use Dronekit-Python API for all useful commands (rc override, change mode, arm drone,...)
        """
        parser = argparse.ArgumentParser()
        parser.add_argument('--connect',
                            default='127.0.0.1:14550')  # Set startup argument --connect with SITL-Output on UDP
        args = parser.parse_args()

        # Connect to the Drone
        self.progress("Connect Dronekit to Drone on: %s" % args.connect)
        vehicle = connect(args.connect, baud=57600, wait_ready=True)
        self.progress("Dronekit connected")
        return vehicle

    def _launch_apm(self):
        """
        purpose: Function to launch SITL resp. sim_vehicle.py
        param:
        return:
        """
        sim_vehicle_py = str(
            os.environ["ARDUPILOT_PATH"]) + "/Tools/autotest/sim_vehicle.py"
        subprocess.Popen(["xterm", "-e", sim_vehicle_py, "-j4", "-v", "ArduCopter", "-f", "gazebo-iris",
                          "--console"])  # add "--console" for Copter GUI
        # subprocess.Popen([sim_vehicle_py,"-j4","-v","ArduCopter","-f","gazebo-iris","--console"])

        RED = '\033[91m'
        BOLD = '\033[1m'
        ENDC = '\033[0m'
        LINE = "%s%s##############################################################################%s" % (
        RED, BOLD, ENDC)
        msg = "\n%s\n" % (LINE)
        msg += "%sLoad Copter parameters in MavProxy console (sim_vehicle.py):%s\n\n" % (BOLD, ENDC)
        msg += "MAV> param load %s\n\n" % (str("~/gym-ws/gym-ws/gym_ws/envs/params/IRIS.param"))
        msg += "%sThen, press <Enter> to launch Gazebo...%s\n\n%s" % (BOLD, ENDC, LINE)
        raw_input(str(msg))

    def send_attitude_target(self, roll_angle=0.0, pitch_angle=0.0, yaw_angle=None, yaw_rate=0.0, use_yaw_rate=False,
                             thrust=0.5):
        """
        purpose: Send an specified attitude message to the flight controller. For more information, see:
        https://mavlink.io/en/messages/common.html#SET_ATTITUDE_TARGET
        param:
            use_yaw_rate: the yaw can be controlled using yaw_angle OR yaw_rate.
                      When one is used, the other is ignored by Ardupilot.
            thrust: 0 <= thrust <= 1, as a fraction of maximum vertical thrust.
                Note that as of Copter 3.5, thrust = 0.5 triggers a special case in
                the code for maintaining current altitude.
            roll_angle:
            pitch_angle:
            yaw_angle:
        return:
        """
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

    def set_attitude(self, roll_angle=0.0, pitch_angle=0.0, yaw_angle=None, yaw_rate=0.0, use_yaw_rate=False,
                     thrust=0.5, duration=0):
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

    def to_quaternion(self, roll=0.0, pitch=0.0, yaw=0.0):
        """
        purpose: Convert degrees to quaternions to avoid gimbal-lock
        param:
            roll:
            pitch:
            yaw:
        return:
            w:
            x:
            y:
            z:
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

    def _takeoff(self, altitude):
        """
        purpose: Takeoff for arming and takeoff to a specified altitude
        param
            altitude:
        return:
        """
        self.progress("Basic pre-arm checks")
        # Don't try to arm until autopilot resp. SITL is ready
        while not self.vehicle.is_armable:
            self.progress("Waiting for Drone to initialise...")
            time.sleep(0.5)
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
            time.sleep(0.5)
        self.progress("Changed mode to %s" % self.vehicle.mode.name)

        while not self.vehicle.armed:
            self.progress("Waiting for arming...")
            self.vehicle.armed = True
            time.sleep(1)
        self.progress("Armed")

        self.progress("Taking off!")
        while True:
            current_altitude = self.vehicle.location.global_relative_frame.alt
            self.vehicle.channels.overrides[3] = 1580
            if current_altitude == altitude * 0.4:
                self.progress("Altitude: %f  Desired: %f" % (current_altitude, altitude))
            # Break and return from function just below target altitude.
            if current_altitude >= altitude * 0.95:  # Check that vehicle has reached takeoff altitude
                self.progress("Reached target altitude")
                self.progress("Begin hovering")
                self.vehicle.channels.overrides[3] = 1500
                break
            elif current_altitude >= altitude * 0.6:
                self.vehicle.channels.overrides[3] = 1530
                self.progress("Altitude: %f  Desired: %f" % (current_altitude, altitude))
                time.sleep(0.2)
            # time.sleep(5)
        """
        Change mode to ALT_HOLD with throttle / thrust on PWM 1500,
        that triggers a special case in the ardupilot code for maintaining current altitude in ALT_HOLD mode
        for the simplest possible hover-example, where the RL has only to control Roll & Pitch
        """

        self.vehicle.channels.overrides[3] = 1500  # In Ardupilot SITL PWM~1500 is the throttle-mid-position
        desired_mode = 'ALT_HOLD'
        self.vehicle.mode = VehicleMode(desired_mode)
        while self.vehicle.mode != desired_mode:
            self.vehicle.mode = VehicleMode(desired_mode)
            time.sleep(0.2)
        self.vehicle.channels.overrides[3] = 1500
        self.progress("Changed mode to %s" % self.vehicle.mode.name)

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def step(self, action):
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except rospy.ServiceException as e:
            print ("/gazebo/unpause_physics service call failed")
            self.progress(action)

        if action == 0:  # FORWARD : {Roll, Pitch, Throttle, Yaw}
            self.vehicle.channels.overrides = {'1': 1500, '2': 1470, '3': 1500, '4': None}
        elif action == 1:  # LEFT
            self.vehicle.channels.overrides = {'1': 1470, '2': 1500, '3': 1500, '4': None}
        elif action == 2:  # RIGHT
            self.vehicle.channels.overrides = {'1': 1530, '2': 1500, '3': 1500, '4': None}
        elif action == 3:  # BACKWARDS
            self.vehicle.channels.overrides = {'1': 1500, '2': 1530, '3': 1500, '4': None}

        observation = self._get_position()

        dist = self.center_distance()
        done = dist > self.max_distance

        if done:
            reward = -100
        else:
            reward = 10 - dist * 8

        return observation, reward, done, {}

    def _killall(self, process_name):
        pids = subprocess.check_output(["pidof", process_name]).split()
        for pid in pids:
            os.system("kill -9 " + str(pid))

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
            # print("Initial latitude : %f, Initial Longitude : %f" % (self.initial_latitude, self.initial_longitude,))

        # print("Current latitude : %f, Current Longitude : %f" % (self.current_latitude, self.current_longitude,))

        self.diff_latitude = self.current_latitude - self.initial_latitude
        self.diff_longitude = self.current_longitude - self.initial_longitude

        print("Diff latitude: %f, Diff Longitude: %f" % (self.diff_latitude, self.diff_longitude,))
        # time.sleep(1) # not sure if this is very helpful, but it should reduce the spam
        return self.diff_latitude, self.diff_longitude

    def center_distance(self):
        """
        purpose: Returns the distance from the Drone to the origin: c = (x^2+y^2)^1/2
        param:
        return: distance
        """
        # distance = math.sqrt(self.diff_latitude ** 2 + self.diff_longitude ** 2)
        return math.sqrt(self.diff_latitude ** 2 + self.diff_longitude ** 2)

    def reset(self):
        """
        purpose: Resets the state of the environment and returns an initial observation
        param:
        return: function _get_position()
        """
        # rospy.wait_for_service('/gazebo/reset_world')  # Resets the model's poses
        # rospy.wait_for_service('/gazebo/reset_simulation') # Resets the entire simulation including the time
        try:
            # reset_proxy.call()
            self.reset_proxy()
        except rospy.ServiceException as e:
            print ("/gazebo/reset_world service call failed")

        # Relaunch autopilot
        # self._relaunch_apm() # It takes far too long to relaunch APM and connect it again, maybe necessary for difficult Situations
        # self._takeoff(5) # Commended out to speedup the episodes

        self.initial_latitude = None
        self.initial_longitude = None

        return self._get_position()
