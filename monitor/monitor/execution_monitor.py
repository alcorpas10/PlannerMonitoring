import rclpy
from rclpy.node import Node

from mutac_msgs.msg import Alarm, State, Plan, Identifier, Label #, DroneRequest, UserResponse, DroneComms
from mutac_msgs.srv import UpdatePlan

from geometry_msgs.msg import PoseStamped, Pose, Twist, TwistStamped
from sensor_msgs.msg import BatteryState

from monitor.monitor_data import State as MonitorState

from monitor.drone import Drone


class ExecutionMonitor(Node):
    def __init__(self, id):
        super().__init__('execution_monitor_' + str(id),
                        allow_undeclared_parameters=True,
                        automatically_declare_parameters_from_overrides=True)
        self.id = id

        self.n_drones = self.get_parameter('n_drones').value
        self.mode = self.get_parameter('mode').value
        if type(self.n_drones) is not int:
            self.get_logger().error('Missing parameter \'n_drones\'')
            exit(1)
        
        self.dist_trj = 5
        self.dist_wp = 2

        homebase = self.get_parameter('homebase.drone'+str(self.id+1)).value
        self.drone = Drone(self.id, homebase)
        self.drone.init_other_drones(self.n_drones)

        self.initializePublishers()
        self.initializeSubscribers()

        self.replan_client = self.create_client(UpdatePlan, '/mutac/update_plan')

        self.timer_period = 0.5 #TODO the rate was 3, now is 2
        self.timer = self.create_timer(self.timer_period, self.timerCallback)


    def initializePublishers(self):
        self.event_pub = self.create_publisher(State, '/mutac/drone_events', 100)
        self.covered_pub = self.create_publisher(Identifier, '/mutac/covered_points', 100)
        #self.drone_request = self.create_publisher(DroneRequest, '/mutac/drone_request', 100)
        #self.comms_pub = self.create_publisher(DroneComms, '/mutac/drone_comms', 100)

    def initializeSubscribers(self):
        if self.mode == 0:
            self.pose_subs = []
            for i in range(self.n_drones):
                if i != self.id:
                    self.pose_subs.append(self.create_subscription(Pose, '/mutac/drone'+str(i+1)+'/drone_pose', self.drone.other_drones[i].positionCallback, 100))
            self.pose_sub = self.create_subscription(Pose, '/mutac/drone'+str(self.id+1)+'/drone_pose', self.drone.positionCallback, 100)
            #self.twist_sub = self.create_subscription(Twist, '/mutac/drone'+str(self.id)+'/drone_twist', self.drone.velocityCallback, 100)
            self.alarm_sub = self.create_subscription(Alarm, '/mutac/drone_alarm', self.alarmCallback, 100)
            self.battery_sub = self.create_subscription(BatteryState, '/mutac/drone'+str(self.id)+'/battery_state', self.drone.batteryCallback, 100)

        elif self.mode == 1:
            uav_name = "drone_sim_"
            # uav_name = "cf"
            for i in range(self.n_drones):
                if i != self.id:
                    self.pose_subs.append(self.create_subscription(PoseStamped, uav_name+str(i)+'/self_localization/pose', 
                                                                   lambda msg: self.drone.other_drones[i].positionCallback(msg.pose), 100))
            uav_name = "drone_sim_"+str(self.id)
            # uav_name = "cf" + str(self.id)
            self.pose_sub = self.create_subscription(PoseStamped, uav_name+'/self_localization/pose', lambda msg: self.drone.positionCallback(msg, self.mode), 100)
            #self.twist_sub = self.create_subscription(TwistStamped, uav_name+'/self_localization/twist', self.drone.velocityCallback, 100)
            #self.alarm_sub = self.create_subscription(Alarm, '/mutac/drone_alarm', self.alarmCallback, 100) # Doesn't exist in aerostack
            self.battery_sub = self.create_subscription(BatteryState, uav_name+'/sensor_measurements/battery', self.drone.batteryCallback, 100)

        self.trj_sub = self.create_subscription(Plan, '/mutac/real_planned_paths', self.trajectoryCallback, 100)
        self.covered_sub = self.create_subscription(Identifier, '/mutac/covered_points', self.waypointCallback, 100)
        self.events_sub = self.create_subscription(State, '/mutac/drone_events', self.eventCallback, 100)
        #self.user_sub = self.create_subscription(UserResponse, '/mutac/user_response', self.responseCallback, 100)
        #self.comms_sub = self.create_subscription(DroneComms, '/mutac/drone_comms', self.commsCallback, 100)

    def timerCallback(self):
        """At a certain rate it is checked if the mission is going ok"""
        event_id = self.drone.checkDrone(self.dist_trj, self.dist_wp)

        if event_id == 0: # MISSION_FINISHED
            pos = self.drone.pos_in_trj if self.drone.deviated else self.drone.position
            msg = State()
            msg.identifier.natural = self.id
            msg.state = event_id
            msg.position.x = pos[0]
            msg.position.y = pos[1]
            msg.position.z = pos[2]
            self.event_pub.publish(msg)
            self.drone.reset()

        elif event_id == 1: # LOST
            pos = self.drone.pos_in_trj if self.drone.deviated else self.drone.position
            msg = State()
            msg.identifier.natural = self.id
            msg.state = event_id
            msg.position.x = pos[0]
            msg.position.y = pos[1]
            msg.position.z = pos[2]
            self.event_pub.publish(msg)
            self.askReplan() # TODO call updatePlan service
            self.drone.waypoints = []

        elif event_id == 2: # LANDED
            msg = State()
            msg.identifier.natural = self.id
            msg.state = event_id
            self.event_pub.publish(msg)

        elif event_id == 3: # WAYPOINT
            msg = Identifier()
            msg.natural = self.id
            self.covered_pub.publish(msg)

        elif event_id == 4: # RECOVERED
            msg = State()
            msg.identifier.natural = self.id
            msg.state = State.RECOVERED
            self.event_pub.publish(msg)
            self.askReplan()

    def waypointCallback(self, msg):
        drone_id = msg.natural
        if drone_id != self.id:
            self.drone.advanceOtherWP(msg.natural)

    def eventCallback(self, msg):
        drone_id = msg.identifier.natural
        if drone_id in self.drone.other_drones.keys() or drone_id in self.drone.lost_drones.keys() or drone_id == self.id:
            if drone_id != self.id:
                if msg.state == State.FINISHED:
                    self.drone.setState(drone_id, MonitorState.MISSION_FINISHED)
                    self.drone.advanceOtherWP(drone_id)
                elif msg.state == State.LANDED:
                    self.drone.setState(drone_id, MonitorState.LANDED)
                elif msg.state == State.LOST:
                    #self.drone.saveState(drone_id)
                    self.drone.setPosInTrj(drone_id, (msg.position.x, msg.position.y, msg.position.z))
                    self.drone.setState(drone_id, MonitorState.LOST)
                elif msg.state == State.RECOVERED: # RECOVERED
                    self.drone.resetDrone(drone_id)
        else:
            self.get_logger().info("WARNING: Event received for a drone that doesn't exists.")

    def trajectoryCallback(self, msg):
        self.get_logger().info("********************")
        # self.get_logger().info("Trajectory received")
        # self.get_logger().info("********************")
        for path in msg.paths:
            self.get_logger().info("Path "+str(path.identifier.natural) + ": "+str(path.points[0]))
            self.get_logger().info("********************")
            self.drone.setWaypoints(path)
    
    def askReplan(self):
        if len(self.drone.other_drones) > 0:
            srv = UpdatePlan.Request()
            srv.plan.paths = self.drone.generatePlanPaths()

            while not self.replan_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('Service not available, waiting again...')
            
            self.replan_client.call_async(srv)

    def alarmCallback(self, msg):
        if msg.identifier.natural == self.id:
            if msg.alarm == Alarm.CAMERA_FAILURE:
                self.drone.camera = False
                self.askReplan() # TODO call updatePlan service       


class GetParam(Node):
    def __init__(self):
        super().__init__('GetParam',
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True)

def main(args=None):
    rclpy.init(args=args)

    id = GetParam().get_parameter('drone_id').value

    exec_mon = ExecutionMonitor(id)
    rclpy.spin(exec_mon)

if __name__ == '__main__':
    main()