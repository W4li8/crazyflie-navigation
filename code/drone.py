# import numpy as np
# from numpy.lib.financial import rate
# from cflib.utils import uri_helper
import cflib.crtp

from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.position_hl_commander import PositionHlCommander
# from cflib.positioning.motion_commander import MotionCommander
from cflib.utils.multiranger import Multiranger

from logger import Logger

import threading, time, random

class Drone():

    def __init__(self, drone_uri, init_pose, flight_altitude, cruise_speed):

        cflib.crtp.init_drivers(enable_debug_driver=False)

        self.crazyflie = SyncCrazyflie(drone_uri)
        self.logger = Logger(self.crazyflie.cf)
        self.crazyflie.open_link()  # auto closes upon program exit

        # setup position controller
        self.hlcontroller = PositionHlCommander(self.crazyflie, x=init_pose[0], y=init_pose[1], z=init_pose[2],
                                              default_height=flight_altitude, default_velocity=cruise_speed)

        # setup sensors
        self.multiranger = Multiranger(self.crazyflie)
        self.multiranger.start()

        self.pose_correction = [-item for item in init_pose]

    def get_pose(self):
        out = [0,0,0,0,0,0]
        for i in range(6):
            out[i]= self.logger.estimated_pose[i] - self.pose_correction[i]
        return tuple(out)

    def get_multiranger(self):
        return (self.multiranger.front, self.multiranger.back, self.multiranger.left, self.multiranger.right)

    def detected_platform(self):
        return random.random() % 2

    def exit(self):
        self.crazyflie.close_link()

    def print_pose(self):
        pose = self.get_pose()
        out = "x = " + str(round(pose[0],3)) + "\n"
        out = out + "y = " + str(round(pose[1],3)) + "\n"
        out = out + "z = " + str(round(pose[2],3)) + "\n"
        out = out + "roll = " + str(round(pose[3],3)) + "\n"
        out = out + "pitch = " + str(round(pose[4],3)) + "\n"
        out = out + "yaw = " + str(round(pose[5],3))
        return out

    def print_multiranger(self):
        out = "front = " + str(round(self.multiranger.front,3)) + "\n"
        out = out + "back = " + str(round(self.multiranger.back,3)) + "\n"
        out = out + "left = " + str(round(self.multiranger.left,3)) + "\n"
        out = out + "right = " + str(round(self.multiranger.right,3))
        return out

    def print_battery(self):
        return "battery voltage = " + str(round(self.logger.battery_voltage,3))
