import math
import time

from monitor.monitor_data import MonitorData, State


class Drone(MonitorData):
    def __init__(self, id, homebase):
        super().__init__(id)
        self.battery = 100
        self.homebase = tuple(homebase)
        self.camera = True
        self.deviated = False
        self.in_homebase = False

        # self.waypoints = []
        # self.inspection_wps = [] # TODO not used

        self.last_distance = float("inf") # prev_distance in local_drone
        self.last_wp = self.position # lastWP in local_drone
        self.time_last_msg = time.time() # from global_drone

        self.other_drones = {} # TODO change name
        self.lost_drones = {}


    def init_other_drones(self, n_drones):
        for i in range(n_drones):
            if i != self.id:
                self.other_drones[i] = MonitorData(i)

    def checkDrone(self, dist_trj, dist_wp):
        if self.state == State.LOST and self.in_homebase:
            self.reset()
            return 4
        
        if self.state == State.NOT_STARTED or self.state == State.LOST:
            return -1

        if not self.camera or self.battery <= 5:
            self.state = State.LOST
            return 1

        wps = self.waypoints

        waypoint_dist = self.distance(self.position, wps[0]['point'])

        # When the drone is in the last waypoint
        if len(wps) == 1 and waypoint_dist <= dist_wp:
            self.advanceWP()

            if self.state == State.GOING_HOME:
                self.state = State.LANDED
                return 2
            else:
                self.state = State.GOING_HOME
                return 0

        # When the drone is in a waypoint
        if waypoint_dist < dist_wp:
            self.advanceWP()
            return 3

        # When the drone has deviated from the trajectory
        if self.distanceToTrj(self.position, self.last_wp, wps[0]['point']) > dist_trj:
            print("Drone: ", self.id, " has deviated from the trajectory")
            print("Position: ", self.position, " Last WP: ", self.last_wp, " Next WP: ", wps[0]['point'], " Distance: ", self.distanceToTrj(self.position, self.last_wp, wps[0]['point']))
            self.pos_in_trj = self.calculateProjection(self.position, self.last_wp, wps[0]['point'])
            self.deviated = True
            self.state = State.LOST
            return 1

        # When the drone has deviated from the trajectory
        if waypoint_dist > self.last_distance and abs(waypoint_dist - self.last_distance) > dist_trj:
            print("Drone: ", self.id, " has deviated from the trajectory")
            print("Waypoint distance: ", waypoint_dist, " Last distance: ", self.last_distance)
            self.deviated = True
            self.state = State.LOST
            return 1

        # When the drone is in the trajectory
        if waypoint_dist < self.last_distance:
            self.last_distance = waypoint_dist
            self.pos_in_trj = self.position

        return -1
    
    def distance(self, p1, p2):
        return self.vectorNorm(p1[0] - p2[0], p1[1] - p2[1], p1[2] - p2[2])
    
    def vectorNorm(self, x, y, z):
        return math.sqrt(x**2 + y**2 + z**2)
    
    def distanceToTrj(self, pos, p1, p2):
        AB = (p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2])
        AP = (pos[0] - p1[0], pos[1] - p1[1], pos[2] - p1[2])

        i = AB[1] * AP[2] - AP[1] * AB[2]
        j = AP[0] * AB[2] - AB[0] * AP[2]
        k = AB[0] * AP[1] - AB[1] * AP[0]
        
        return self.vectorNorm(i, j, k) / self.vectorNorm(AB[0], AB[1], AB[2])
    
    def calculateProjection(self, pos, wp1, wp2):
        AM = (pos[0] - wp1[0], pos[1] - wp1[1], pos[2] - wp1[2])
        u = (wp2[0] - wp1[0], wp2[1] - wp1[1], wp2[2] - wp1[2])

        scalar = (AM[0] * u[0] + AM[1] * u[1] + AM[2] * u[2]) / (self.vectorNorm(u[0], u[1], u[2])**2)
        
        return (wp1[0] + scalar * u[0], wp1[1] + scalar * u[1], wp1[2] + scalar * u[2])
    
    def setWaypoints(self, path):
        path_id = path.identifier.natural
        # if the id is of a lost drone something is wrong
        if path_id == self.id:
            super().setWaypoints(path)
            self.last_wp = self.position
            self.last_distance = float("inf")
        else:
            self.other_drones[path_id].setWaypoints(path)

    # def setInspectionWPS(self, inspection_wps):
    #     self.inspection_wps = inspection_wps[1:]

    def advanceWP(self):
        self.last_distance = float("inf")
        self.last_wp = self.waypoints[0]['point']
        super().advanceWP()
        # if len(self.inspection_wps) > 0:
        #     self.inspection_wps = self.inspection_wps[1:]

    def advanceOtherWP(self, drone_id):
        self.other_drones[drone_id].advanceWP()

    # def saveState(self, drone_id):
    #     self.other_drones[drone_id].saveState()

    def setState(self, drone_id, state):
        self.other_drones[drone_id].setState(state)
        if state == State.LOST:
            self.lost_drones[drone_id] = self.other_drones[drone_id]
            self.other_drones.pop(drone_id)

    def setPosInTrj(self, drone_id, pos):
        self.other_drones[drone_id].pos_in_trj = pos

    # def checkConnection(self, time_threshold):
    #     time_pose = time.time() - self.time_last_msg
    #     if self.state != State.NOT_STARTED and time_pose > time_threshold:
    #         self.state = State.LOST

    def batteryCallback(self, msg):
        self.battery = msg.percentage
    
    def positionCallback(self, msg):
        super().positionCallback(msg)
        self.time_last_msg = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.in_homebase = True if self.distance(self.position, self.homebase) < 0.2 else False        

    # def generatePlanPaths(self):
    #     paths = []

    #     for drone_id, drone in self.other_drones.items():
    #         path = drone.generatePlanPath()
    #         #print("Path of drone " + str(drone_id) + ": " + str(path))
    #         if path is not None:
    #             paths.append(path)
        
    #     path = self.generatePlanPath()
    #     #print("My path " + str(path))
    #     paths.append(path)
    #     return paths

    def resetDrone(self, drone_id):
        if drone_id in self.lost_drones.keys():
            self.lost_drones[drone_id].reset()
            self.other_drones[drone_id] = self.lost_drones[drone_id]
            self.lost_drones.pop(drone_id)
        else:
            self.other_drones[drone_id].reset()

    def reset(self):
        super().reset()
        
        self.battery = 100 # TODO check how the real battery works
        self.camera = True
        self.deviated = False

        self.waypoints = []
        # self.inspection_wps = []

        self.last_distance = float("inf")
        self.last_wp = self.position

        # for _, drone in self.other_drones.items(): # TODO maybe reset drones when each one finishes
        #    drone.reset()
