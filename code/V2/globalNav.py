import numpy as np

class GLOBALNAV(object):
    """docstring for GlobalNav."""

    def __init__(self, world_dimensions, home_area, dest_area, tile_size):
        self.sizeX = world_dimensions[0]
        self.sizeY = world_dimensions[1]
        self.stepsize = tile_size[0] # 10cm step size

        self.home_origin = home_area[0]
        self.dest_origin = dest_area[0]

        self.potential_loc = 'dest'
        self.potential_origin = []
        self.potential_end = []
        self.potential_weight = 0.05
        self._setPotential()

        self.mode = 'navigate' # among navigate, coverage, disabled
        self.coverage_vector = [0,1]

        self.obstacles = [] # store global pos of obstacles
        self.obstacles_weight = 2
        self.obstacles_spread = 50

    def _setPotential(self):
        print(f"Resetting potential to : {self.potential_loc}")
        if self.potential_loc == 'dest':
            self.potential_origin = self.dest_origin
        elif self.potential_loc == 'home':
            self.potential_origin = self.home_origin
        self.potential_origin = [item + delta for item, delta in zip(self.potential_origin, 2*[self.stepsize])]
        self.potential_end = [item + delta for item, delta in zip(self.potential_origin, [1.5 - self.stepsize , 3 - self.stepsize])]

    def _obstacle_potential(self, x, y, o):
        return self.obstacles_weight*np.exp(-self.obstacles_spread*((x - o[0])**2 + (y - o[1])**2))

    def _obstacle_gradient(self, x, y, o):
        coeff = self.obstacles_weight * self.obstacles_spread
        gradx = -2 * coeff * (x - o[0]) * self._obstacle_potential(x,y,o)
        grady = -2 * coeff * (y - o[1]) * self._obstacle_potential(x,y,o)
        return np.array([gradx, grady])

    def _main_gradient(self, x, y):
        return self.potential_weight*np.array([2*(x - self.potential_origin[0]), 2*(y - self.potential_origin[1])])

    def computeNormalizedGradient(self, pos, mag=False):
        x,y = pos
        grad = self._main_gradient(x, y)
        for o in self.obstacles:
            grad += self._obstacle_gradient(x, y, o)
        grad = grad/np.linalg.norm(grad)
        return grad

    def signalObstacles(self, pose, rangers):
        yaw = pose[-1]
        for i, r in enumerate(rangers):
            if r != -1:
                X = round(pose[0] + r*np.cos(yaw + i * np.pi/2),1)
                Y = round(pose[1] + r*np.sin(yaw + i * np.pi/2))
                if (X > 0 and X < self.sizeX) and (Y > 0 and Y < self.sizeY):
                    item = [X,Y]
                    if item not in self.obstacles:
                        self.obstacles.append(item)

    def getWaypoint(self, pos): #pose as X,Y,something
        waypoint = None
        # print(self.mode + " "*40 + "\r")
        if self.mode == 'navigate': #navigation mode to traverse the map
            if np.linalg.norm( np.array(pos) - np.array(self.potential_origin) ) < self.stepsize:
                self.mode = 'coverage'
                waypoint = self.potential_origin
            else:
                grad = self.computeNormalizedGradient(pos)
                waypoint = self.stepsize * grad
                waypoint = [pos[0] - waypoint[0], pos[1] - waypoint[1]]

        elif self.mode == 'coverage': # coverage mode to explore the map
            if np.linalg.norm(np.array(pos) - np.array(self.potential_end)) < self.stepsize:
                print('Reaching end of area coverage')
                self.mode = 'disabled'
                return pos
            if pos[1] <= self.stepsize/2:
                self.coverage_vector = [1, 1]
            elif pos[1] >= self.sizeY - self.stepsize/2:
                self.coverage_vector = [1, -1]
            else:
                self.coverage_vector[0] = 0
            waypoint = [pos[0] + self.stepsize*self.coverage_vector[0],\
                        pos[1] + self.stepsize*self.coverage_vector[1]]

        elif self.mode == 'disabled':
            waypoint = pos
            self.potential_loc = "home"
            self._setPotential()
            self.mode = 'navigate'

        # print(f"got {pos}, return {waypoint}")
        return waypoint

    def resetNavigation(self):
        self.mode = 'disabled'
        self.obstacles = []
























# EOF
