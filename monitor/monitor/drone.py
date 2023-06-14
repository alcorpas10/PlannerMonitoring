import math
import time

from monitor.drone_data import DroneData, State


class Drone(DroneData):
    def __init__(self, id, homebase, dist_hb, max_time_stopped):
        """Initializes the drone object"""
        super().__init__(id)
        self.homebase = tuple(homebase)
        self.dist_hb = dist_hb
        self.max_time_stopped = max_time_stopped

        self.battery = 100
        self.camera_ok = True
        self.deviated = False
        self.stopped = False
        self.in_homebase = False
        self.repeat = False
        self.cancelled = False

        self.last_distance = float("inf")
        self.last_wp = self.position
        self.time_last_msg = time.time() # Not used by now
        self.time_last_wp = None

        self.covered_distance = 0
        self.inspection_height = None


    def checkDrone(self, dist_trj, dist_wp):
        """Checks the drone status and returns the corresponding event code. Have into 
        account that the order of the code in this method is important"""
        # When the drone reaches the homebase after getting lost
        if self.state == State.LOST and self.in_homebase:
            print("Drone ", self.id, " was recovered")
            self.reset()
            return 4
        
        # When the user cancels the drone's mission
        if self.cancelled:
            print("Drone's ", self.id, " mission was cancelled")
            if self.state == State.ON_MISSION:
                self.state = State.LOST
            self.cancelled = False
            return 6
        
        # When the drone did not start a mission or got lost
        if self.state == State.NOT_STARTED or self.state == State.LOST:
            return -1

        # When the drone stops for too long with a mission to complete
        if self.time_last_wp is not None and time.time() - self.time_last_wp > self.max_time_stopped:
            print("Drone ", self.id, " stopped for too long during the mission")
            # self.state = State.LOST
            # self.stopped = True
            # return 1
            return -1 # Uncomment the previous lines and remove this one to activate stop detection
        
        # When the drone camera is broken
        if not self.camera_ok:
            print("Drone ", self.id, " has a broken camera")
            self.state = State.LOST
            return 1
        
        # When the drone has very low battery
        if self.battery <= 5:
            print("Drone ", self.id, " has very low battery")
            self.state = State.LOST
            return 1

        # When the drone has to repeat the previous waypoint
        if self.repeat:
            print("Drone ", self.id, " needs to repeat the last waypoint")
            self.repeat = False
            return 5
        
        wps = self.waypoints

        waypoint_dist = self.distance(self.position, wps[0]['point'])
        
        # When the drone is in the last waypoint
        if len(wps) == 1 and waypoint_dist <= dist_wp:
            self.advanceWP()
            self.state = State.GOING_HOME
            return 0

        # When the drone is in a waypoint
        if waypoint_dist < dist_wp:
            self.advanceWP()
            return 3

        # When the drone has deviated from the trajectory
        if self.distanceToTrj(self.position, self.last_wp, wps[0]['point']) > dist_trj:
            self.deviated = True
            print("Drone ", self.id, " has deviated from the trajectory")
            print("Position: ", self.position, " Last WP: ", self.last_wp, " Next WP: ", wps[0]['point'], " Distance: ", self.distanceToTrj(self.position, self.last_wp, wps[0]['point']))
            self.pos_in_trj = self.calculateProjection(self.position, self.last_wp, wps[0]['point'])
            self.state = State.LOST
            return 1

        # When the drone has deviated from the trajectory
        if waypoint_dist > self.last_distance and abs(waypoint_dist - self.last_distance) > dist_trj:
            self.deviated = True
            print("Drone ", self.id, " has deviated from the trajectory")
            print("Waypoint distance: ", waypoint_dist, " Last distance: ", self.last_distance)
            self.state = State.LOST
            return 1

        # When the drone is in the trajectory as expected
        if waypoint_dist < self.last_distance:
            self.last_distance = waypoint_dist
            self.pos_in_trj = self.position

        return -1
    
    def distance(self, p1, p2):
        """Returns the distance between two points"""
        return self.vectorNorm(p1[0] - p2[0], p1[1] - p2[1], p1[2] - p2[2])
    
    def vectorNorm(self, x, y, z):
        """Returns the norm of a vector"""
        return math.sqrt(x**2 + y**2 + z**2)
    
    def distanceToTrj(self, pos, p1, p2):
        """Returns the distance between a point and a line (the trajectory)"""
        AB = (p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2])
        AP = (pos[0] - p1[0], pos[1] - p1[1], pos[2] - p1[2])

        i = AB[1] * AP[2] - AP[1] * AB[2]
        j = AP[0] * AB[2] - AB[0] * AP[2]
        k = AB[0] * AP[1] - AB[1] * AP[0]
        
        return self.vectorNorm(i, j, k) / self.vectorNorm(AB[0], AB[1], AB[2])
    
    def calculateProjection(self, pos, wp1, wp2):
        """Returns the projection of a point in a line"""
        AM = (pos[0] - wp1[0], pos[1] - wp1[1], pos[2] - wp1[2])
        u = (wp2[0] - wp1[0], wp2[1] - wp1[1], wp2[2] - wp1[2])

        scalar = (AM[0] * u[0] + AM[1] * u[1] + AM[2] * u[2]) / (self.vectorNorm(u[0], u[1], u[2])**2)
        
        return (wp1[0] + scalar * u[0], wp1[1] + scalar * u[1], wp1[2] + scalar * u[2])
    
    def setWaypoints(self, path):
        """Saves the waypoints of the path that the drone is going to follow"""
        path_id = path.identifier.natural
        if path_id == self.id:
            super().setWaypoints(path)
            self.last_wp = self.position
            self.last_distance = float("inf")
            self.time_last_wp = time.time()

    def advanceWP(self):
        """Advances the drone to the next waypoint"""
        self.covered_distance += self.distance(self.last_wp, self.waypoints[0]['point'])
        self.last_wp = self.waypoints[0]['point']
        self.last_distance = float("inf")
        self.time_last_wp = time.time()
        super().advanceWP()

    def repeatWP(self):
        """Repeats the last covered waypoint. It is used when there is a minor error
        during the drone inspection that does not require a replan"""
        if len(self.waypoints) == 0:
            print("Error: Drone ", self.id, " has no waypoints to repeat")
            return
        self.repeat = True
        self.last_distance = float("inf")
        self.waypoints.insert(0, {'label': self.last_label, 'point': self.last_wp})
        self.last_wp = self.waypoints[1]['point']

    def batteryCallback(self, msg):
        """Callback that updates the drone battery"""
        self.battery = msg.percentage
    
    def positionCallback(self, msg):
        """Callback that updates the drone position"""
        super().positionCallback(msg)
        self.time_last_msg = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.in_homebase = True if self.distance(self.position, self.homebase) < self.dist_hb else False

    def reset(self):
        """Resets the drone to its initial state"""
        super().reset()
        
        self.battery = 100 # TODO check how the real battery works
        self.camera_ok = True
        self.deviated = False
        self.stopped = False
        self.cancelled = False

        self.last_distance = float("inf")
        self.last_wp = self.position
        self.time_last_wp = None
