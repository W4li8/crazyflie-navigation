import json
import time
import threading
import numpy as np

from drone import Drone
from globalNav import GLOBALNAV


class Pilot:
    def __init__(self, mission_file):
        """ Read mission_file formatted in json as detailed in user_help.txt
            - mission_file: string with path to mission file
        """
        with open(mission_file, mode="r") as j_object:
            data = json.load(j_object)

        # self.init_reset = True
        self.emergency_stop = False

        # flight settings
        self.drone_uri = data["drone_uri"]
        self.drone_pose = data["init_pose"]
        self.flight_altitude = data["flight_altitude"]
        self.cruise_speed = data["cruise_speed"]
        self.manoeuver_speed = data["manoeuver_speed"]
        self.safety_distance = data["safety_distance"]
        self.platform_dim = data["platform_dimensions"]

        # init drone
        self.drone = Drone(
            self.drone_uri,
            self.drone_pose,
            self.flight_altitude,
            self.cruise_speed
        )
        # init navigation
        self.navigation = GLOBALNAV(
            data["world_dimensions"],
            data["home_area"],
            data["dest_area"],
            data["tile_size"],
        )
        # environment analysis thread flags
        self.analyze_environment = None  # active or inactive, manipulated at takeoff and landing
        self.platform_edge_detected = None

        self.moving_dict = {"front" : self.drone.hlcontroller.forward,
                       "back"  : self.drone.hlcontroller.back,
                       "left"  : self.drone.hlcontroller.left,
                       "right" : self.drone.hlcontroller.right}

        self.reverse_dict = {"front" : "back",
                        "back"  : "front",
                        "left"  : "right",
                        "right" : "left"}

    def get_map_pose(self):
        """ Update and Return current pose estimate in real world coordinates """
        self.drone_pose = self.drone.get_pose()
        return self.drone_pose

    def take_off(self):
        """ Take off and start thread for environment analysis """

        self.drone.reset_estimator()

        self.drone.hlcontroller.take_off()
        self.drone_pose = self.drone.get_pose()

        self.analyze_environment = True
        threading.Thread(target=self.environment_analysis_thread).start()

    def _find_boundary(self, mode):
        delta = 0.02
        sleeping = 0.1
        w = 2
        dist =  0

        self.platform_edge_detected = False
        while not self.platform_edge_detected:
            self.moving_dict[mode](delta, velocity=self.manoeuver_speed)
            dist += delta
            if dist > w * self.platform_dim[1]:
                self.moving_dict[self.reverse_dict[mode]](dist, velocity=self.manoeuver_speed)
                dist = 0
                self.platform_edge_detected = False

        pos = self.drone.get_pose()[:2]
        while self.platform_edge_detected:
            time.sleep(sleeping)
        self.moving_dict[self.reverse_dict[mode]](dist, velocity=self.manoeuver_speed)
        while self.platform_edge_detected:
            time.sleep(sleeping)

        print(pos)
        return dist, np.array(pos)

    def _is_platform(self, l, r, f, b):
        def _error(a,b, expected, epsilon):
            print(np.abs(np.abs(a-b) - expected))
            if np.abs(np.abs(a-b) - expected) > expected * epsilon :
                return True
            else:
                return False

        if _error(l[0], r[0], self.platform_dim[0], epsilon=0.707) or \
            _error(f[1], b[1], self.platform_dim[0], epsilon=0.707):
                return False
        else:
            return True


    def land(self, centering=True):
        time.sleep(1)
        if centering:
            for _ in range(3):
                while self.platform_edge_detected:
                    time.sleep(0.1)

                dist_left, pos_left = self._find_boundary("left")
                dist_right, pos_right = self._find_boundary("right")
                total = dist_left + dist_right
                print("Centering along L-R axis")
                self.drone.hlcontroller.left(total/2 - dist_left, velocity=0.5*self.manoeuver_speed)


                dist_front, pos_front = self._find_boundary("front")
                dist_back, pos_back = self._find_boundary("back")
                print("centering on centroid")
                centroid = (pos_left + pos_right + pos_front + pos_back) / 4
                self.fly_to_waypoint(centroid)

                if self._is_platform(pos_left, pos_right, pos_front, pos_back):
                    print('PLATFORM')
                    break

            print("Landing on platform (hopefully)")

        self.analyze_environment = False
        self.drone.hlcontroller.land()
        self.drone_pose = self.drone.get_pose()
        self.platform_edge_detected = False

    def environment_analysis_thread(self):
        """ Analyze environment to detect platform edges, obstacles and the unknown """

        time.sleep(1)

        # initialize altitude measure to steady value
        avg_altitude = 0.00  # average altitude as perceived by multiranger
        nb_measures = 0
        while nb_measures < 200:  # sufficient for decent average
            if self.drone.multiranger.down is not None:
                avg_altitude += self.drone.multiranger.down
                nb_measures += 1
        avg_altitude /= nb_measures

        # infinite loop scanning environment through multiranger measures
        k_z = self.drone.get_pose()[2]
        flt1 = k_z
        prev_flt1 = k_z
        flt2 = 0
        thresh = 1.5e-3

        alpha1 = 5e-2
        alpha2 = 0.1

        kalmanZ = []

        cnt = 0
        while self.analyze_environment:
            cnt += 1
            # look for platform
            curr_altitude = self.drone.multiranger.down
            if curr_altitude is not None and self.navigation.mode == "coverage":
                prev_flt1 = flt1
                flt1 = alpha1*curr_altitude + (1-alpha1)*flt1
                diff = np.abs(flt1 - prev_flt1)
                flt2 = alpha2 * diff + (1-alpha2)*flt2

                # debug
                k_z = self.drone.get_pose()[2]
                kalmanZ.append(k_z)
                np.save("kalmanZ", kalmanZ)

                if flt2 > thresh:
                    self.platform_edge_detected = True
                else:
                    self.platform_edge_detected = False

            # signal obstacle ranges to navigation
            front, back, left, right = self.drone.get_multiranger()
            if front is None or front > self.safety_distance:
                front = -1
            # else: self.obstacle_detected["F"] = True
            if back is None or back > self.safety_distance:
                back = -1
            # else: self.obstacle_detected["B"] = True
            if left is None or left > self.safety_distance:
                left = -1
            # else: self.obstacle_detected["L"] = True
            if right is None or right > self.safety_distance:
                right = -1
            # else: self.obstacle_detected["R"] = True
            self.navigation.signalObstacles(self.drone_pose, (front, back, left, right))

            # safety measure
            dist_unknown = self.drone.multiranger.up
            if dist_unknown is not None and dist_unknown < self.safety_distance:
                print("Flying too close to the unknown, landing asap.")
                self.analyze_environment = False  # proper thread exit
                self.emergency_stop = True
                self.drone.hlcontroller.land()

            time.sleep(0.1)  # slow down refresh rate

    def fly_mission_thread(self): # maybe useless now without GUI
        threading.Thread(target=self.fly_mission).start()

    def fly_to_waypoint(self, waypoint):
        """ Fly to waypoint and update drone pose
            - waypoint: list [x, y] [m, m]
        """
        self.drone.move_to(waypoint)
        self.drone_pose = self.drone.get_pose()

    def follow_navigation(self):
        """ Follow navigation waypoints until platform detected """

        data = []
        i = 0
        while not self.platform_edge_detected and not self.emergency_stop:
            i += 1
            self.fly_to_waypoint(self.navigation.getWaypoint(self.drone_pose[:2]))
            data.append(self.drone.get_pose())
            if i % 2 == 0:
                np.save("data", data)

    def fly_mission(self):
        """ Crazyflie project main FSM, with safe exit in case of mishap """

        try:
            print("Fly mission")
            for mission_step in ("Go to destination", "Come back home"):
                if not self.emergency_stop:
                    print(mission_step)
                    # perform flight
                    print("before take off")
                    self.take_off()
                    print("before follow navigation")
                    self.follow_navigation()
                    print("before landing")
                    self.land()
                    # signal arrival
                    self.navigation.resetNavigation()  # maybe mission_step could be passed as param
                    time.sleep(2)

        except Exception as exception:
            print("An error occurred - exiting asap.", exception)
            self.analyze_environment = False  # proper thread exit
            self.emergency_stop = True
            self.drone.hlcontroller.land()

    def skills_test(self):
        """ Try out pilot capabilities, autonomous mission, manual control, sensor readings, quit available """

        while True:
            cmd = input("skills test action ")
            if cmd == "FM":
                self.fly_mission()

            # manual control options
            elif cmd == "T":
                self.drone.hlcontroller.take_off()
                self.drone_pose = self.drone.get_pose()
            elif cmd == "W":
                self.drone.hlcontroller.forward(0.1)  # , velocity=self.cruise_speed)
                self.drone_pose = self.drone.get_pose()
            elif cmd == "S":
                self.drone.hlcontroller.back(0.1)  # , velocity=self.cruise_speed)
                self.drone_pose = self.drone.get_pose()
            elif cmd == "A":
                self.drone.hlcontroller.left(0.1)  # , velocity=self.cruise_speed)
                self.drone_pose = self.drone.get_pose()
            elif cmd == "D":
                self.drone.hlcontroller.right(0.1)  # , velocity=self.cruise_speed)
                self.drone_pose = self.drone.get_pose()
            elif cmd == "Y":
                self.drone.hlcontroller.up(0.1)  # , velocity=self.cruise_speed)
                self.drone_pose = self.drone.get_pose()
            elif cmd == "X":
                self.drone.hlcontroller.down(0.1)  # , velocity=self.cruise_speed)
                self.drone_pose = self.drone.get_pose()
            elif cmd == "L":
                self.drone.hlcontroller.land()
                self.drone_pose = self.drone.get_pose()

            # specific algorithms
            elif cmd == "C1":
                self.take_off()
            elif cmd == "C2":  # platform centering
                self.land()

            elif cmd == "O":  # obstacle avoidance
                self.take_off()
                #! other actions needed here

            # access variables
            elif cmd == "P":  # expected drone pose [x, y, z, yaw]
                print(self.drone_pose[:3], self.drone_pose[-1])
            elif cmd == "R":  # multiranger values
                print(
                    f"RangerU {self.drone.multiranger.up} RangerD {self.drone.multiranger.down}"
                    f"RangerL {self.drone.multiranger.left} RangerR {self.drone.multiranger.right}"
                    f"RangerF {self.drone.multiranger.front} RangerB {self.drone.multiranger.back}"
                )

            # quit skills test
            elif cmd == "Q":
                self.analyze_environment = False  # proper thread exit
                self.drone.hlcontroller.land()  # just in case
                break
            else:
                print("Invalid input")

            print(self.drone_pose[:3], self.drone_pose[-1])

    def exit(self):
        self.emergency_stop = True
        self.analyze_environment = False
        self.drone.exit()
