from enum import Enum

from mutac_msgs.msg import LabeledPath, LabeledPoint


class State(Enum):
    NOT_STARTED = 0
    ON_MISSION = 1
    MISSION_FINISHED = 2
    GOING_HOME = 3
    LANDED = 4
    LOST = 5

class Label(Enum):
    POSITIONING_LABEL = 0
    COVERING_LABEL = 1

class MonitorData:
    def __init__(self, id):
        self.id = id

        self.state = State.NOT_STARTED
        # self.last_state = State.NOT_STARTED # TODO check if necessary to avoid replannings

        self.position = (0.0, 0.0, 0.0)
        self.pos_in_trj = (0.0, 0.0, 0.0)

        self.waypoints = []

        self.last_label = Label.POSITIONING_LABEL
        

    def setState(self, state):
        self.state = state
        if state == State.LOST:
            self.waypoints = []

    def setWaypoints(self, path):
        """Set the waypoints of the drone starting from the second point
        of the path. The first point is the current position of the drone."""
        self.state = State.ON_MISSION
        self.waypoints = []
        for l_point in path.points[1:]:
            label = l_point.label.natural
            point = (l_point.point.x, l_point.point.y, l_point.point.z)
            self.waypoints.append({'label': label, 'point': point})

    def advanceWP(self):
        """Remove the first waypoint left of the drone."""
        self.last_label = self.waypoints[0]['label']
        self.waypoints = self.waypoints[1:]

    def generatePlanPath(self):
        """Generate the path of the drone to be sent to the planner.
        The path is composed by the current position of the drone and
        the left waypoints of the drone."""
        path = LabeledPath()
        path.identifier.natural = self.id if self.state != State.LOST else -1

        point = LabeledPoint()
        pos = self.position if self.state != State.LOST else self.pos_in_trj
        point.point.x = pos[0]
        point.point.y = pos[1]
        point.point.z = pos[2]

        # TODO maybe check if waypoints are empty
        for waypoint in self.waypoints:
            l_point = LabeledPoint()
            l_point.point.x = waypoint['point'][0]
            l_point.point.y = waypoint['point'][1]
            l_point.point.z = waypoint['point'][2]
            l_point.label.natural = waypoint['label']
            path.points.append(l_point)

        #point.label.natural = Label.POSITIONING_LABEL if len(path.points) <= 0 or path.points[0].label.natural != self.last_label else path.points[0].label.natural
        if len(path.points) <= 0 or path.points[0].label.natural != self.last_label:
            point.label.natural = Label.POSITIONING_LABEL.value
        else:
            point.label.natural = path.points[0].label.natural
                
        path.points.insert(0, point)
        return path

    def positionCallback(self, msg):
        self.position = (msg.position.x, msg.position.y, msg.position.z)

    def reset(self): # TODO check if there are more things to reset 
        self.state = State.NOT_STARTED

        self.waypoints = []
        #self.position = (0.0, 0.0, 0.0)
        #self.pos_in_trj = (0.0, 0.0, 0.0)
