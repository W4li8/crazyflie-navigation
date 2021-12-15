import numpy as np
import threading
from functools import partial

import sys,time

class GUI(object):
    """docstring for Plotter."""

    def __init__(self, ui, pilot):
        self.pilot = pilot
        self.drone_pose = [0,0,0,0,0,0] # as x,y,yaw
        self.drone_rangers = [0,0,0,0]

        self.label = ui.label
        self.canva_map = ui.canva_map.canvas
        self.canva_pot = ui.canva_pot.canvas
        self._clearPlot(self.canva_map)
        self._clearPlot(self.canva_pot)
        self.plotPotential()
        self.last_number_of_obstacles = -1

        self.drone_body = self.canva_map.ax.plot(-1,-1,'ro') #marker='o',c='r')
        # self.drone_direction = self.canva_map.ax.arrow(0,0,0,1,head_width=0.05)
        self.plot_thread = None

    def _clearPlot(self, canva):
        canva.ax.clear()
        canva.ax.set_xlim([0,self.pilot.navigation.sizeX])
        canva.ax.set_ylim([0,self.pilot.navigation.sizeY])
        canva.ax.set_xlabel('X [m]')
        canva.ax.set_ylabel('Y [m]')
        canva.ax.grid(True)
        if canva == self.canva_map:
            canva.ax.set_title('Arena for the drone')
        else:
            canva.ax.set_title('Potential field for the drone')

        canva.ax.axvline(x=1.5) #start area
        canva.ax.axvline(x=3.5) #goal area

    def updateDrone(self):
        self.drone_pose = self.pilot.drone.get_pose()
        self.drone_rangers = self.pilot.drone.get_multiranger()

    def plot_update_thread(self):
        if self.plot_thread is not None:
            self.plot_thread.join()
        self.plot_thread = threading.Thread(target=self.plotUpdate)
        self.plot_thread.start()

    def plotUpdate(self):
        self._clearPlot(self.canva_map)
        self.updateDrone()
        self.plotDrone()
        # self.plotObstacles()
        self.canva_map.draw()

        # if len(self.pilot.navigation.obstacles) != self.last_number_of_obstacles:
        # self.canva_pot.ax.clear()
        # self.last_number_of_obstacles = len(self.pilot.navigation.obstacles)
        # self.plotPotential()
        # self.canva_pot.draw()
        

    def plotObstacles(self):
        for o in self.pilot.navigation.obstacles:
            self.canva_map.ax.scatter(o[0], o[1], marker='.', c='k')

    def plotDrone(self):
        # print(self.drone_pose[:2])
        self.drone_body = self.canva_map.ax.scatter(self.drone_pose[0],self.drone_pose[1],marker='o', color='r')
        # self.canva_map.ax.arrow(self.drone_pose[0],self.drone_pose[1],0.1*np.cos(self.drone_pose[-1]),\
        #                     0.1*np.sin(self.drone_pose[-1]),head_width=0.05)

        # plot rangers (optional)
        # for i in range(4):
        #     X = [self.drone_pose[0], self.drone_pose[0] + self.drone_rangers[i]*np.cos(self.drone_pose[2] + i * np.pi/2)]
        #     Y = [self.drone_pose[1], self.drone_pose[1] + self.drone_rangers[i]*np.sin(self.drone_pose[2] + i * np.pi/2)]
        #     self.canva_map.ax.plot(X,Y)
        self.label.setText(f"Rangers | F : {self.drone_rangers[0]} | B : {self.drone_rangers[1]} | L : {self.drone_rangers[2]} | R : {self.drone_rangers[3]}")

    def plotPotential(self):
        x, y = np.meshgrid(np.linspace(0,5,25), np.linspace(0,3,25))
        pot = self.pilot.navigation.potential(x,y, symbolic=False)
        levels = np.arange(0,20,0.2)
        self.canva_pot.ax.contour(x, y, pot, levels=levels)

