import argparse, json
import time, threading
import numpy as np

from enum import Enum, auto
from drone import Drone
from global_nav import GlobalNav


from global_display import GlobalDisplay as disp

class MissionState(Enum):

    Idle = auto()  # patience padawan
    GoFindTarget = auto() # aka GoOrderPizza
    ComeBackHome = auto() # aka BringMyPizza


class Pilot:

    def __init__(self, mission_file):
        """ Read mission_file formatted in json as detailed in user_help.txt """
        with open(mission_file, mode="r") as j_object:
            data = json.load(j_object)

        # fun debug parameters
        self.mission_id = data["mission_id"]
        self.name = data["pilot_name"]

        # flight settings
        self.flight_altitude = data["flight_altitude"]
        self.cruise_speed    = data["cruise_speed"]
        self.centering_speed = data["centering_speed"]
        self.safety_distance = data["safety_distance"]
        self.ranger_distance = data["ranger_distance"]
        self.platform_size   = data["platform_size"]

        self.edge = False
        self.rangerdown = []
        self.last_rangerdown = None # store the last reading of z ranger

        self.obstacle_activated = False
        self.obstacle_near = False

        self.world_dim = data["world_dimensions"]
        self.nav_settings = data["nav_settings"]
        self.tile_size = data["tile_size"]

        # init modules
        self.drone = Drone(data["drone_uri"], data["init_pose"], self.flight_altitude, self.cruise_speed)
        self.globalnav = GlobalNav(500,300,50,30,100,100,True)

        #! additional dictionary of flight settings we can add later
        print(data["mission_settings"])

    def takeoff(self):
        self.drone.hlcontroller.take_off(height=self.flight_altitude, velocity=0.2)

    def land(self):
        dist_left = 0
        dist_right = 0
        dist_front = 0
        dist_back = 0

        delta = 0.02
        sleeping = 0.5
        w = 2

        # centering left/right
        self.edge = False
        while not self.edge:
            self.drone.hlcontroller.left(delta, velocity=self.centering_speed)
            dist_left += delta
            if dist_left > w*self.platform_size:
                self.drone.hlcontroller.right(dist_left, velocity=self.centering_speed)
                dist_left = 0
                self.edge = False
        time.sleep(sleeping)
        self.drone.hlcontroller.right(dist_left, velocity=0.5*self.centering_speed)
        time.sleep(sleeping)

        self.edge = False
        while not self.edge:
            self.drone.hlcontroller.right(delta, velocity=self.centering_speed)
            dist_right += delta
            if dist_right > w*self.platform_size:
                self.drone.hlcontroller.left(dist_right, velocity=self.centering_speed)
                dist_right = 0
                self.edge = False
        time.sleep(sleeping)

        total = dist_left + dist_right
        print('Centering along L-R axis')
        self.drone.hlcontroller.left(total/2, velocity=0.5*self.centering_speed)
        time.sleep(sleeping)

        # centering front/back
        self.edge = False
        while not self.edge:
            self.drone.hlcontroller.forward(delta, velocity=self.centering_speed)
            dist_front += delta
            if dist_front > w*self.platform_size:
                self.drone.hlcontroller.back(dist_front, velocity=self.centering_speed)
                dist_front = 0
                self.edge = False
        time.sleep(sleeping)
        self.drone.hlcontroller.back(dist_front, velocity=0.5*self.centering_speed)
        time.sleep(sleeping)

        self.edge = False
        while not self.edge:
            self.drone.hlcontroller.back(delta, velocity=self.centering_speed)
            dist_back += delta
            if dist_back > w*self.platform_size:
                self.drone.hlcontroller.forward(dist_back, velocity=self.centering_speed)
                dist_back = 0
                self.edge = False
        time.sleep(sleeping)
        total = dist_front + dist_back
        print('Centering along F-B axis')
        self.drone.hlcontroller.forward(total/2, velocity=0.5*self.centering_speed)

        print('Landing on platform (hopefully)')
        # print(self.drone.hlcontroller.get_position())
        self.drone.hlcontroller.land()

    def edgeDetection(self):
        threshold = 0.01
        temp = 0
        cnt = 0
        for _ in range(1000):
            val = self.drone.multiranger.down
            if val is not None:
                temp += val
                cnt += 1
        self.last_rangerdown = temp / cnt

        while self.drone.hlcontroller._is_flying:
            new = self.drone.multiranger.down
            if new is not None and new != self.last_rangerdown:
                if new - self.last_rangerdown > threshold:
                    self.edge = True
                    print(f'{time.time()} : EDGE')
                self.last_rangerdown = new


    # def regulateYaw(self):
        # self.drone.hlcontroller._hl_commander.go_to(0,0,self.flight_altitude, 0, 1)

    def fly_to_waypoint(self, waypoint):
        self.drone.hlcontroller.go_to(waypoint[0], waypoint[1])


    def fly_mission(self):
        """ Sequential logic to the Crazyflie Pizza project """
        # Go order pizza
        """
        self.set_objective(MissionState.GoFindTarget)
        self.takeoff()
        self.follow_navigation()
        self.land()
        self.set_objective(MissionState.Idle)

        time.sleep(3) # stop for visual effect


        self.set_objective(MissionState.ComeBackHome)
        self.takeoff()
        self.follow_navigation()
        self.land()
        self.set_objective(MissionState.Idle)
        """
        self.globalnav.cover_area(0,0,10,10)
        self.follow_navigation()
        


    def set_objective(self, mission_state):
        if mission_state == MissionState.Idle:
            return
        elif mission_state == MissionState.GoFindTarget:
            # find platform in area x: 3.5m -> 5m, y: 0m -> 3m
            c1, r1, c2, r2 = np.array(self.nav_settings["food_area"]) // self.tile_size
            self.globalnav.cover_area(c1, r1, c2, r2)
        elif mission_state == MissionState.ComeBackHome:
            # find platform in area x: 0m -> 1.5m, y: 0m -> 3m
            c1, r1, c2, r2 = np.array(self.nav_settings["home_area"]) // self.tile_size
            self.globalnav.cover_area(c1, r1, c2, r2)

    def follow_navigation(self):
        # determine sequence of actions and coordinate Drone with GlobalNav module
        # handles obstacle management
        dt = 1
        while not self.detected_platform():
            st = time.time()
            waypoint = self.globalnav.get_waypoint(self.drone.get_pose(),self.drone.get_multiranger(),dt)
            print(self.drone.get_pose()[0],self.drone.get_pose()[1])
            print([waypoint[0]/100,waypoint[1]/100])
#            disp.showGrid(self.globalnav,confidenceAsAlpha=True)
            #self.fly_to_waypoint([waypoint[0]/100,waypoint[1]/100])
            time.sleep(0.1)
            dt = time.time() - st
            #self.register_surroundings(self.drone.get_pose(), self.drone.get_multiranger(), dt)

    def detected_platform(self):
        return self.edge



    def skills_test(self):
        # self.signal_obstacles()
        self.running = True
        z_ranger = []

        while(self.running):
            z_ranger.append(self.drone.multiranger.down)

            cmd = input("action ")
            if cmd == "T":
                self.takeoff()
                time.sleep(1)

                thread_edges = threading.Thread(target=self.edgeDetection)
                thread_edges.start()

            elif cmd == 'W':
                self.drone.hlcontroller.forward(0.1, velocity=self.cruise_speed)
            elif cmd == 'S':
                self.drone.hlcontroller.back(0.1, velocity=self.cruise_speed)
            elif cmd == 'A':
                self.drone.hlcontroller.left(0.1, velocity=self.cruise_speed)
            elif cmd == 'D':
                self.drone.hlcontroller.right(0.1, velocity=self.cruise_speed)
            elif cmd == 'Y':
                self.drone.hlcontroller.up(0.1, velocity=self.cruise_speed)
            elif cmd == 'X':
                self.drone.hlcontroller.down(0.1, velocity=self.cruise_speed)
            
            
            elif cmd == 'Z':
                correction = list(self.drone.get_pose())
                correction[0] = correction[0] - self.globalnav.drone_x/100
                correction[1] = correction[1] - self.globalnav.drone_y/100
                self.drone.corrected_pose = correction

            elif cmd == 'E':
                while not self.obstacle_near and self.edge == False:
                    self.drone.hlcontroller.forward(0.02, velocity=self.cruise_speed)

                if self.edge == True:
                    time.sleep(0.5)
                    self.land()

            elif cmd == "Y":

                pos = self.drone.get_pose()
                measured_x = pos[0] * 100
                measured_y = pos[1] * 100
                self.globalnav.update(measured_x,measured_y,1)
                # self.globalnav.sense(pos,self.drone.get_multiranger())


            elif cmd == "L":
                self.drone.hlcontroller.land()
            elif cmd == "C":
                self.land()
            elif cmd == "F":
                self.fly_mission()

            elif cmd == "Q":
                self.obstacle_activated = False
                self.drone.hlcontroller.land()
                self.drone.exit()
                break
            elif cmd == "R":
                while(1):
                    print(f"RangerU {self.drone.multiranger.up} RangerD {self.drone.multiranger.down} "\
                            f"RangerL {self.drone.multiranger.left} RangerR {self.drone.multiranger.right} "\
                            f"RangerF {self.drone.multiranger.front} RangerB {self.drone.multiranger.back}")
            else:
                print("Invalid input")


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
    crazypilot = Pilot(args.input).skills_test() # warning: crazypilot = None
    tf = time.time()

    print(f"Done. Total mission time was {int(tf-ti)} s.")