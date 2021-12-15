from PyQt5 import QtWidgets, QtCore, QtGui
from PyQt5.QtCore import Qt

from pyqtgraph import PlotWidget, plot
import pyqtgraph as pg
import sys  # We need sys so that we can pass argv to QApplication
import os
from random import randint

import networkx as nx

from global_nav import GlobalNav
from main import Pilot

import math




def colorFromSemantics(inp,conf):
    if inp == "clear" :
        return QtGui.QColor(255*(1-conf) + conf*0, 255*(1-conf) + conf*255, 255*(1-conf) + conf*0)
    if inp == "explored" :
        return QtGui.QColor(255*(1-conf) + conf*255, 255*(1-conf) + conf*100, 255*(1-conf) + conf*100)
    if inp == "blocked" :
        return QtGui.QColor(255*(1-conf) + conf*255, 255*(1-conf) + conf*0, 255*(1-conf) + conf*0)
    if inp == "unexplored" :
        return QtGui.QColor(255*(1-conf) + conf*0, 255*(1-conf) + conf*0, 255*(1-conf) + conf*255)
    if inp == "unreachable" :
        return QtGui.QColor(255*(1-conf) + conf*80, 255*(1-conf) + conf*80,255*(1-conf) + conf*80)
    if inp == "forbidden" :
        return QtGui.QColor(255*(1-conf) + conf*255,255*(1-conf) + conf* 0,255*(1-conf) + conf*255)




class WorldDisplay(QtWidgets.QWidget):

    def __init__(self, drone, nav, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.setSizePolicy(
            QtWidgets.QSizePolicy.MinimumExpanding,
            QtWidgets.QSizePolicy.MinimumExpanding
        )

        self.drone = drone
        self.nav = nav

        self.ox = 100
        self.oy = 100

    def sizeHint(self):
        return QtCore.QSize(700,500)

    # used by the widget to handle redrawing itself
    def paintEvent(self, e):
        # print("PAINT :)")
        painter = QtGui.QPainter(self)

        # set the background:
        brush = QtGui.QBrush()
        brush.setColor(QtGui.QColor(255, 255, 255))
        brush.setStyle(Qt.SolidPattern)
        rect = QtCore.QRect(0, 0, painter.device().width(), painter.device().height())
        painter.fillRect(rect, brush)


        #constants : 
        nav = self.nav
        sx = (nav.width  / nav.cols)  # spacing between nodes on the x axis
        sy = (nav.height / nav.rows)  # spacing between nodes on the y axis


        #draw grid cells
        for node in nav.G.nodes:
            cx = nav.G.nodes[node]['obj'].x+self.ox
            cy = nav.G.nodes[node]['obj'].y+self.oy
            state = nav.G.nodes[node]['obj'].state

            brush.setColor(colorFromSemantics(state,nav.G.nodes[node]['obj'].confidence))
            # brush.setColor(QtGui.QColor(255, 0, 0))
            brush.setStyle(Qt.SolidPattern)
            rect = QtCore.QRect(cx-sx/2, cy-sy/2, sx, sy)
            painter.fillRect(rect, brush)


        painter.setPen(QtGui.QPen(Qt.black,  1, Qt.SolidLine))
        painter.drawRect(self.ox, self.oy, 500, 300)



        #plotting drone position and sensor values
        

        drone_x = self.drone.get_pose()[0]*100+self.ox
        drone_y = self.drone.get_pose()[1]*100+self.oy
        yaw = (self.drone.get_pose()[-1]/180)*math.pi


        path = QtGui.QPainterPath()
        path.moveTo(drone_x+8*math.cos(yaw+2*math.pi/3),drone_y+8*math.sin(yaw+2*math.pi/3))
        path.lineTo(drone_x+8*math.cos(yaw),drone_y+8*math.sin(yaw))
        path.lineTo(drone_x+8*math.cos(yaw-2*math.pi/3),drone_y+8*math.sin(yaw-2*math.pi/3))
        painter.setBrush(QtCore.Qt.blue)
        painter.drawPath(path)

        painter.setPen(QtGui.QPen(Qt.red,  2, Qt.DotLine))

        front_sensor = self.drone.get_multiranger()[0]*100
        painter.drawLine(drone_x, drone_y, drone_x+ math.cos(yaw) * front_sensor, drone_y +math.sin(yaw) * front_sensor)
        back_sensor = self.drone.get_multiranger()[1]*100
        painter.drawLine(drone_x, drone_y, drone_x+ math.cos(yaw+math.pi) * back_sensor, drone_y +math.sin(yaw+math.pi) * back_sensor)
        left_sensor = self.drone.get_multiranger()[2]*100
        painter.drawLine(drone_x, drone_y, drone_x+ math.cos(yaw+math.pi/2) * left_sensor, drone_y +math.sin(yaw+math.pi/2) * left_sensor)
        right_sensor = self.drone.get_multiranger()[3]*100
        painter.drawLine(drone_x, drone_y, drone_x+ math.cos(yaw-math.pi/2) * right_sensor, drone_y +math.sin(yaw-math.pi/2) * right_sensor)

        #plotting next waypoint and final waypoint
        next_waypoint = nav.next_waypoint
        if next_waypoint != None:
            painter.setPen(QtGui.QPen(Qt.black,  2, Qt.DotLine))
            painter.drawLine(next_waypoint[0]+self.ox-5, next_waypoint[1]+self.oy, next_waypoint[0]+self.ox+5, next_waypoint[1]+self.oy)
            painter.drawLine(next_waypoint[0]+self.ox, next_waypoint[1]+self.oy-5, next_waypoint[0]+self.ox, next_waypoint[1]+self.oy+5)

        grid_goal_waypoint = nav.goal_waypoint
        if grid_goal_waypoint != None:

            goal_waypoint = nav.gridToSpace(grid_goal_waypoint[0],grid_goal_waypoint[1])        
            painter.setPen(QtGui.QPen(Qt.black,  2, Qt.DotLine))
            painter.drawLine(goal_waypoint[0]+self.ox-5, goal_waypoint[1]+self.oy, goal_waypoint[0]+self.ox+5, goal_waypoint[1]+self.oy)
            painter.drawLine(goal_waypoint[0]+self.ox, goal_waypoint[1]+self.oy-5, goal_waypoint[0]+self.ox, goal_waypoint[1]+self.oy+5)

        #plotting the path : 
        if self.nav.path != None:
            for i in range(0,len(self.nav.path)-1):
                painter.setPen(QtGui.QPen(Qt.blue,  2, Qt.DotLine))
                [x1,y1] = nav.gridToSpace(self.nav.path[i][0],self.nav.path[i][1])        
                [x2,y2] = nav.gridToSpace(self.nav.path[i+1][0],self.nav.path[i+1][1])        
                painter.drawLine(x1+self.ox, y1+self.oy, x2+self.ox, y2+self.oy)
        
        #plotting the trace : 
        if self.nav.trace != None:
            for i in range(0,len(self.nav.trace)-1):
                painter.setPen(QtGui.QPen(Qt.red,  1, Qt.SolidLine))
                [x1,y1] = nav.gridToSpace(self.nav.trace[i][0],self.nav.trace[i][1])        
                [x2,y2] = nav.gridToSpace(self.nav.trace[i+1][0],self.nav.trace[i+1][1])        
                painter.drawLine(x1+self.ox, y1+self.oy, x2+self.ox, y2+self.oy)
            

        painter.end()
 
    def _trigger_refresh(self):
        self.update()