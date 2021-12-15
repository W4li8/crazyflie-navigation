#!/usr/bin/env python3

from PyQt5 import QtWidgets, QtCore, QtGui
from pyqtgraph import PlotWidget, plot
import pyqtgraph as pg
import sys, os
from random import randint

import argparse, json, time, threading
import numpy as np

from enum import Enum, auto
from drone import Drone
from global_nav import GlobalNav

from pilot import Pilot

from world_display import WorldDisplay

class MissionState(Enum):

    Idle = auto()  # patience padawan
    GoFindTarget = auto() # aka GoOrderPizza
    ComeBackHome = auto() # aka BringMyPizza

class MainWindow(QtWidgets.QMainWindow):

    #initializes the window
    def __init__(self, file, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)

        with open(file, mode="r") as j_object:
            data = json.load(j_object)

        self.label_update_period = data["label_update_period"]
        self.controller_update_period = data["controller_update_period"]

        #defining our main widget and layout
        self.main_widget = QtWidgets.QWidget(self)
        self.setCentralWidget(self.main_widget)
        self.layout = QtWidgets.QVBoxLayout(self.main_widget)

        self.setWindowTitle("Mission controller")

        #define our graph plotting widget
        # self.graphWidget = pg.PlotWidget()
        # self.layout.addWidget(self.graphWidget)
        # self.graphWidget.setBackground('w')
        self.displayWidget = WorldDisplay(crazypilot.drone,crazypilot.globalnav)
        self.layout.addWidget(self.displayWidget)

        # define our text output widget
        self.out_text = QtWidgets.QLabel("drone position")
        self.layout.addWidget(self.out_text)

        self.auto_update = False    # if True, live update the map
        self.nav_autority = False   # if True follow global nav waypoints, if False, fly manual (only follows waypoints when computed, so when auto_update is true)

#        pen = pg.mkPen(color=(255, 0, 0))
#        self.data_line =  self.graphWidget.plot(self.x, self.y, pen=pen)

        # set labels update timer
        self.timer = QtCore.QTimer()
        self.timer.setInterval(self.label_update_period)
        self.timer.timeout.connect(self.labelUpdate)
        self.timer.start()

        # set canva update time
        self.f_timer = QtCore.QTimer()
        self.f_timer.setInterval(self.controller_update_period)
        self.f_timer.timeout.connect(self.flightUpdate)
        self.f_timer.start()

    def labelUpdate(self):
        self.out_text.setText("drone state : \n" + crazypilot.drone.print_pose() + "\nmultiranger : \n" + str(crazypilot.drone.print_multiranger()) + "\n" + crazypilot.drone.print_battery())
        self.displayWidget._trigger_refresh()

    def flightUpdate(self):
        if self.auto_update:
            crazypilot.globalnav.next_waypoint = self.nav_event()
            # print(crazypilot.globalnav.next_waypoint)
            if self.nav_autority:
                crazypilot.waypoint(crazypilot.globalnav.next_waypoint[0],crazypilot.globalnav.next_waypoint[1])

    def mouseReleaseEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton:
            print("Update waypoint")
            mx = event.pos().x()
            my = event.pos().y()
            rx = mx - 13 - self.displayWidget.ox
            ry = my - 13 - self.displayWidget.oy
            print(rx,ry)
            crazypilot.globalnav.goal_waypoint = crazypilot.globalnav.spaceToGrid(rx,ry)


    def nav_event(self):
        pos = crazypilot.drone.get_pose()
        sensor = crazypilot.drone.get_multiranger()
        delta_t = self.controller_update_period/1000
        return crazypilot.globalnav.get_waypoint(pos,sensor,delta_t)

    def keyPressEvent(self, event):  # reads keyboard events
        key = event.key()

        if key == QtCore.Qt.Key_T:
            print("Takeoff")
            crazypilot.takeoff()

            thread_edges = threading.Thread(target=crazypilot.edgeDetection)
            thread_edges.start()
            self.nav_event()

        elif key == QtCore.Qt.Key_Q:
            print("Exiting...")
            crazypilot.drone.hlcontroller.land()
            crazypilot.drone.exit()

        elif key == QtCore.Qt.Key_W:
            print("translating forward")
            crazypilot.drone.hlcontroller.forward(0.1, velocity=crazypilot.cruise_speed)
            self.nav_event()
        elif key == QtCore.Qt.Key_S:
            print("translating backward")
            crazypilot.drone.hlcontroller.back(0.1, velocity=crazypilot.cruise_speed)
            self.nav_event()
        elif key == QtCore.Qt.Key_A:
            print("translating left")
            crazypilot.drone.hlcontroller.left(0.1, velocity=crazypilot.cruise_speed)
            self.nav_event()
        elif key == QtCore.Qt.Key_D:
            print("translating right")
            crazypilot.drone.hlcontroller.right(0.1, velocity=crazypilot.cruise_speed)
            self.nav_event()

        elif key == QtCore.Qt.Key_C:
            print("centering on platform")
            crazypilot.land()

        elif key == QtCore.Qt.Key_L:
            print("landing (automatically shuts down auto_update)")
            crazypilot.drone.hlcontroller.land()
            self.auto_update = False
            crazypilot.globalnav.path = None

        elif key == QtCore.Qt.Key_Z:
            print("zero position")
            # correction = list(crazypilot.drone.get_pose())
            # correction[0] = correction[0] - crazypilot.globalnav.drone_x/100
            # correction[1] = correction[1] - crazypilot.globalnav.drone_y/100
            # crazypilot.drone.pose_correction = correction
            # print(correction)

        elif key == QtCore.Qt.Key_Y:
            print("forced sensing event")
            self.nav_event()

        elif key == QtCore.Qt.Key_U:
            self.auto_update = not self.auto_update
            print(f"auto update : {self.auto_update}")
            if not self.auto_update:
                crazypilot.globalnav.path = None

        elif key == QtCore.Qt.Key_J:
            self.nav_autority = not self.nav_autority
            print(f"global nav autority : {self.nav_autority}")

if __name__ == '__main__':
    print("Ground Control Station running. Wait for the drone to connect")

    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--input", type=str, required=True, \
                        help="required mission file formatted as explained in user_help.txt")
    parser.add_argument("-d", "--debug", type=bool, \
                        help="option to activate debug code sections")
    parser.add_argument("-l", "--logs", type=str, \
                        help="option to activate logging, specify log file")
    args = parser.parse_args()

    ti = time.time()
    crazypilot = Pilot(args.input) # warning: crazypilot = Nonesafa

    # instantiating the QT app, in turn, this will plot logger and global nav data
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow(args.input)
    window.show()

    #crazypilot.skills_test()
    sys.exit(app.exec_())

    tf = time.time()
    print(f"Done. Total mission time was {int(tf-ti)} s.")
