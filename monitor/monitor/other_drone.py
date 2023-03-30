import time
from monitor.monitor_data import MonitorData, State, Label


class OtherDrone(MonitorData):
    def __init__(self, id):
        super().__init__(id)
        #self.time_last_msg = time.time()
        #self.last_label = Label.POSITIONING_LABEL
        self.last_state = self.state # TODO check if there is no problem with this
        self.waypoints = []


    def saveState(self):
        self.last_state = self.state

    def checkConnection(self, time_threshold):
        time_pose = time.time() - self.time_last_msg
        if self.state != State.NOT_STARTED and time_pose > time_threshold:
            self.state = State.LOST

    def positionCallback(self, msg):
        super().positionCallback(msg)
        self.time_last_msg = time.time()

    def reset(self):
        super().reset()
        self.time_last_msg = time.time()
        self.last_label = Label.POSITIONING_LABEL
        self.last_state = self.state # TODO check if there is no problem with this
        self.waypoints = []
