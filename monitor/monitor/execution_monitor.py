import rclpy
from rclpy.node import Node
from rclpy import qos

from mutac_msgs.msg import Alarm, State, Plan, Identifier, DroneComms, UserRequest
from mutac_msgs.srv import Replan, DroneRequest

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Empty, String

from monitor.drone import Drone


class ExecutionMonitor(Node):
    """Class that monitors the execution of the mission. It is in charge of checking the
    status of its drone and sending the corresponding events to the planner and other
    components of the architecture. It also reacts to the received events"""
    def __init__(self, id):
        """Initializes the execution monitor node"""
        super().__init__('execution_monitor_' + str(id),
                        allow_undeclared_parameters=True,
                        automatically_declare_parameters_from_overrides=True)
        self.id = id

        # Monitor constant parameters
        self.dist_trj = self.get_parameter('distance.trajectory').value
        self.dist_wp = self.get_parameter('distance.waypoint').value
        dist_hb = self.get_parameter('distance.homebase').value
        max_time_stopped = self.get_parameter('max_time_stopped').value
        self.error_limit = self.get_parameter('error_limit').value
        self.namespace = self.get_parameter('namespace').value

        # Obtains the homebase position and creates the drone object
        homebase = self.get_parameter('homebase.drone'+str(self.id+1)).value
        self.drone = Drone(self.id, homebase, dist_hb, max_time_stopped)
        self.lost_drones = {}

        # Initializes the ros2 publishers
        self.initializePublishers()

        # Initializes the ros2 subscribers
        self.initializeSubscribers()

        # Initializes the ros2 clients
        self.initializeClients()

        # Initializes the timer
        self.timer_period = 0.25 # The rate is 4 Hz
        self.timer = self.create_timer(self.timer_period, self.timerCallback)


    def initializePublishers(self):
        """Initializes the publishers"""
        self.event_pub = self.create_publisher(State, '/planner/notification/drone_events', qos.QoSProfile(reliability=qos.ReliabilityPolicy.RELIABLE, depth=100))
        self.covered_pub = self.create_publisher(Identifier, '/planner/notification/covered_points', qos.QoSProfile(reliability=qos.ReliabilityPolicy.RELIABLE, depth=100))
        self.comms_pub = self.create_publisher(DroneComms, '/planner/comms/drone_comms', 100)
        self.drone_resp_pub = self.create_publisher(String, '/planner/comms/drone_response', 100)

    def initializeSubscribers(self):
        """Initializes the subscribers. The topics are the ones used in Aerostack. To monitor
        real drones the 'cf' uav_name should be used instead of 'drone_sim' in the namespace"""
        uav_name = self.namespace+str(self.id)

        # Aerostack topics
        self.pose_sub = self.create_subscription(PoseStamped, uav_name+'/self_localization/pose', self.drone.positionCallback, qos.qos_profile_sensor_data)
        self.battery_sub = self.create_subscription(BatteryState, uav_name+'/sensor_measurements/battery', self.drone.batteryCallback, qos.qos_profile_sensor_data)

        # Planner topics
        self.trj_sub = self.create_subscription(Plan, '/planner/planned_paths', self.trajectoryCallback, qos.QoSProfile(reliability=qos.ReliabilityPolicy.RELIABLE, depth=10))
        self.replan_sub = self.create_subscription(Empty, '/planner/replanning/request_wps', self.replanCallback, 100)
        self.alarm_sub = self.create_subscription(Alarm, '/planner/signal/drone_alarm', self.alarmCallback, qos.QoSProfile(reliability=qos.ReliabilityPolicy.RELIABLE, depth=100)) # Not used by now
        self.comms_sub = self.create_subscription(DroneComms, '/planner/comms/drone_comms', self.commsCallback, 100)
        self.user_req_sub = self.create_subscription(UserRequest, '/planner/comms/user_request', self.userRequestCallback, 100)

    def initializeClients(self):
        """Initializes the clients"""
        self.ask_replan_client = self.create_client(Replan, '/planner/replanning/ask_replan')
        self.provide_wp_client = self.create_client(Replan, '/planner/replanning/provide_wps')
        self.drone_request_client = self.create_client(DroneRequest, '/planner/comms/drone_request')

    def timerCallback(self):
        """At a certain rate it is checked the state of the drone along the mission.
        If an event is detected, it is published"""
        event_id = self.drone.checkDrone(self.dist_trj, self.dist_wp)

        if event_id == 0: # MISSION_FINISHED
            msg = Identifier()
            msg.natural = self.id
            self.covered_pub.publish(msg)
            msg = State()
            msg.identifier.natural = self.id
            msg.state = event_id
            self.event_pub.publish(msg)
            self.drone.reset()
            # self.askReplan() # Uncomment to let the drone help once it has finished its mission

        elif event_id == 1: # LOST
            # Publish a message in the drone_events topic
            self.drone.covered_distance += self.drone.distance(self.drone.last_wp, self.drone.position)
            msg = State()
            msg.identifier.natural = self.id
            msg.state = event_id
            if self.drone.deviated or self.drone.stopped:
                msg.type = State.HOMEBASE
                msg.position.x = self.drone.homebase[0]
                msg.position.y = self.drone.homebase[1]
                msg.position.z = self.drone.homebase[2]
            else:
                msg.type = State.LAND                
            self.event_pub.publish(msg)
            # Publish a message in the drone_comms topic
            msg = DroneComms()
            msg.identifier.natural = self.id
            msg.type = DroneComms.LOST
            self.comms_pub.publish(msg)
            self.askReplan()
            self.drone.waypoints = []

        elif event_id == 2: # LANDED
            self.get_logger().info('Drone covered '+str(self.drone.covered_distance)+' meters')
            msg = State()
            msg.identifier.natural = self.id
            msg.state = event_id
            self.event_pub.publish(msg)

        elif event_id == 3: # WP ARRIVED
            msg = Identifier()
            msg.natural = self.id
            self.covered_pub.publish(msg)

        elif event_id == 4: # RECOVERED
            self.get_logger().info('Drone covered '+str(self.drone.covered_distance)+' meters')
            msg = State()
            msg.identifier.natural = self.id
            msg.state = State.RECOVERED
            self.event_pub.publish(msg)
            # self.askReplan() # Uncomment to let the drone help once it is ready to fly again

        elif event_id == 5: # WP REPEATED
            self.drone.covered_distance += self.drone.distance(self.drone.waypoints[0]['point'], self.drone.position)
            msg = State()
            msg.identifier.natural = self.id
            msg.state = State.WP_REPEATED
            pos = self.drone.waypoints[0]['point']
            msg.position.x = pos[0]
            msg.position.y = pos[1]
            msg.position.z = pos[2]
            self.event_pub.publish(msg)

        elif event_id == 6: # CANCELLED
            self.drone.covered_distance += self.drone.distance(self.drone.last_wp, self.drone.position)
            msg = State()
            msg.identifier.natural = self.id
            msg.state = State.LOST
            msg.type = State.HOMEBASE
            msg.position.x = self.drone.homebase[0]
            msg.position.y = self.drone.homebase[1]
            msg.position.z = self.drone.homebase[2]
            self.event_pub.publish(msg)

    def trajectoryCallback(self, msg):
        """Callback for the trajectory topic. It is used to set the waypoints of the drone"""
        self.get_logger().info("********************")
        for path in msg.paths:
            self.get_logger().info("Path "+str(path.identifier.natural) + ": "+str(len(path.points)))
            self.get_logger().info("********************")
            self.drone.setWaypoints(path)

    def replanCallback(self, msg):
        """Callback for the replan topic. When accessed the planner 
        is asking for the left waypoints of the drone to replan the trajectory"""
        path = self.drone.generatePlanPath(False)

        if path != None: # If there are waypoints left
            srv = Replan.Request()
            srv.path = path

            while not self.provide_wp_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn('Service not available, waiting again...')
            
            # The waypoints are sent through the provide_wp service
            self.provide_wp_client.call_async(srv)

    def askReplan(self):
        """Asks for a replan to the planner. It is used when the drone is 
        lost or recovered"""
        srv = Replan.Request()
        srv.path = self.drone.generatePlanPath(True)
        self.get_logger().info("Asking replan: "+str(len(srv.path.points)))

        while not self.ask_replan_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Service not available, waiting again...')

        # The request is sent through the ask_replan service
        self.ask_replan_client.call_async(srv)

    def alarmCallback(self, msg):
        """Callback for the alarm topic. When error detection is done in other node and
        an error must be monitored, it is sent through this topic"""
        self.get_logger().info("Alarm received: "+str(msg))
        if msg.identifier.natural == self.id:
            if msg.alarm == Alarm.CAMERA_FAILURE:
                self.drone.camera_ok = False
            elif msg.alarm == Alarm.PHOTO_ERROR or msg.alarm == Alarm.VIBRATION_ERROR:
                self.drone.repeatWP() # Minor error, the drone repeats the waypoint

    def commsCallback(self, msg):
        """Callback for the treatment of the communications between drones"""
        drone_id = msg.identifier.natural
        self.get_logger().info("Comms received: "+str(msg))
        if msg.type == DroneComms.LOST and drone_id != self.id:
            if drone_id in self.lost_drones.keys():
                self.lost_drones[drone_id] += 1
            else:
                self.lost_drones[drone_id] = 1
            self.get_logger().info("Lost drone: "+str(drone_id)+" Lost drones: "+str(len(self.lost_drones)))
            if len(self.lost_drones) == self.error_limit: # The mission should be cancelled, a request is sent
                srv = DroneRequest.Request()
                srv.type = DroneComms.CANCEL
                self.get_logger().info("Requesting the cancellation of the mission")

                i = 0
                while not self.drone_request_client.wait_for_service(timeout_sec=1.0) and i < 5:
                    self.get_logger().warn('Service not available, waiting again...')
                    i += 1
                
                if i == 3:
                    self.get_logger().warn('Service not available, could not cancel the mission')
                    return
                
                # The request is sent through the drone_request service
                self.drone_request_client.call_async(srv)
                self.get_logger().info("Cancellation requested")

                # Finally a message is published in the drone_comms topic to inform the other drones
                msg = DroneComms()
                msg.identifier.natural = self.id
                msg.type = DroneComms.CANCEL
                self.comms_pub.publish(msg)
                self.lost_drones = {}

        # The request to cancel has been made so the dictionary is reset
        if msg.type == DroneComms.CANCEL and drone_id != self.id:
            self.lost_drones = {}

    def userRequestCallback(self, msg):
        """Callback for the user_request topic. It is used to receive the user requests"""
        drone_id = msg.identifier.natural
        self.get_logger().info("User request received: "+str(msg))
        if drone_id == self.id or drone_id == -1:
            if msg.text.data == 'swarm_state':
                msg = String(data=str(self.id)+': State '+str(self.drone.state))
                self.drone_resp_pub.publish(msg)
            elif msg.text.data == 'cancel_mission':
                self.drone.cancelled = True
                msg = String(data=str(self.id)+': Mission cancelled')
                self.drone_resp_pub.publish(msg)


class GetParam(Node):
    """Auxiliary class to get the drone id from the launch file and
    initialize the monitor node with it"""
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