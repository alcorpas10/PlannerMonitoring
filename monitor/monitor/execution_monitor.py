import rclpy
from rclpy.node import Node
from rclpy import qos

from mutac_msgs.msg import Alarm, State, Plan, Identifier, LabeledPath #, DroneRequest, UserResponse, DroneComms
from mutac_msgs.srv import Replan

from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import BatteryState, Imu
from std_msgs.msg import Empty

from monitor.monitor_data import State as MonitorState

from monitor.drone import Drone


class ExecutionMonitor(Node):
    def __init__(self, id):
        """Initializes the execution monitor node"""
        super().__init__('execution_monitor_' + str(id),
                        allow_undeclared_parameters=True,
                        automatically_declare_parameters_from_overrides=True)
        self.id = id

        # TODO avoid hardcoding, check if can be variable
        # Monitor constant parameters
        self.dist_trj = 0.5
        self.dist_wp = 0.2

        # Obtains the homebase position and creates the drone object
        homebase = self.get_parameter('homebase.drone'+str(self.id+1)).value
        self.drone = Drone(self.id, homebase)

        # Initializes the ros2 publishers and subscribers
        self.initializePublishers()
        self.initializeSubscribers()

        # Initializes the ros2 clients
        self.ask_replan_client = self.create_client(Replan, '/mutac/ask_replan')
        self.provide_wp_client = self.create_client(Replan, '/mutac/provide_wps')

        # Initializes the timer
        self.timer_period = 0.5 # TODO the rate was 3, now is 2
        self.timer = self.create_timer(self.timer_period, self.timerCallback)

        # Opens the file to save the data
        self.file = open("monitor_data.txt", "w")

        # self.max_orientation_x = (-100.0, -100.0, -100.0, -100.0)
        # self.max_orientation_y = (-100.0, -100.0, -100.0, -100.0)
        # self.max_orientation_z = (-100.0, -100.0, -100.0, -100.0)
        # self.max_orientation_w = (-100.0, -100.0, -100.0, -100.0)
        # self.min_orientation_x = (100.0, 100.0, 100.0, 100.0)
        # self.min_orientation_y = (100.0, 100.0, 100.0, 100.0)
        # self.min_orientation_z = (100.0, 100.0, 100.0, 100.0)
        # self.min_orientation_w = (100.0, 100.0, 100.0, 100.0)

        # self.max_angular_velocity_x = (-100.0, -100.0, -100.0)
        # self.max_angular_velocity_y = (-100.0, -100.0, -100.0)
        # self.max_angular_velocity_z = (-100.0, -100.0, -100.0)
        # self.min_angular_velocity_x = (100.0, 100.0, 100.0)
        # self.min_angular_velocity_y = (100.0, 100.0, 100.0)
        # self.min_angular_velocity_z = (100.0, 100.0, 100.0)

        # self.max_linear_acceleration_x = (-100.0, -100.0, -100.0)
        # self.max_linear_acceleration_y = (-100.0, -100.0, -100.0)
        # self.max_linear_acceleration_z = (-100.0, -100.0, -100.0)
        # self.min_linear_acceleration_x = (100.0, 100.0, 100.0)
        # self.min_linear_acceleration_y = (100.0, 100.0, 100.0)
        # self.min_linear_acceleration_z = (100.0, 100.0, 100.0)


    def initializePublishers(self):
        """Initializes the publishers"""
        self.event_pub = self.create_publisher(State, '/mutac/drone_events', qos.QoSProfile(reliability=qos.ReliabilityPolicy.RELIABLE, depth=100))
        self.covered_pub = self.create_publisher(Identifier, '/mutac/covered_points', qos.QoSProfile(reliability=qos.ReliabilityPolicy.RELIABLE, depth=100))
        #self.drone_request = self.create_publisher(DroneRequest, '/mutac/drone_request', 100)
        #self.comms_pub = self.create_publisher(DroneComms, '/mutac/drone_comms', 100)

    def initializeSubscribers(self):
        """Initializes the subscribers. The topics are the ones used in Aerostack. To monitor
        real drones the 'cf' uav_name should be used instead of 'drone_sim'"""
        uav_name = "/drone_sim_"+str(self.id)
        # uav_name = "cf"+str(self.id)

        # Aerostack topics
        self.pose_sub = self.create_subscription(PoseStamped, uav_name+'/self_localization/pose', self.drone.positionCallback, qos.qos_profile_sensor_data)
        #self.twist_sub = self.create_subscription(TwistStamped, uav_name+'/self_localization/twist', self.drone.velocityCallback, 100)
        self.battery_sub = self.create_subscription(BatteryState, uav_name+'/sensor_measurements/battery', self.drone.batteryCallback, qos.qos_profile_sensor_data)
        self.imu_sub = self.create_subscription(Imu, uav_name+'/sensor_measurements/imu', self.imuCallback, qos.qos_profile_sensor_data)

        # Mutac topics
        self.trj_sub = self.create_subscription(Plan, '/mutac/real_planned_paths', self.trajectoryCallback, qos.QoSProfile(reliability=qos.ReliabilityPolicy.RELIABLE, depth=10))
        #self.covered_sub = self.create_subscription(Identifier, '/mutac/covered_points', self.waypointCallback, 100)
        #self.events_sub = self.create_subscription(State, '/mutac/drone_events', self.eventCallback, 100)
        self.replan_sub = self.create_subscription(Empty, '/mutac/request_wps', self.replanCallback, 100) # TODO check the published message
        #self.alarm_sub = self.create_subscription(Alarm, '/mutac/drone_alarm', self.alarmCallback, 100) # Doesn't exist in aerostack
        #self.user_sub = self.create_subscription(UserResponse, '/mutac/user_response', self.responseCallback, 100)
        #self.comms_sub = self.create_subscription(DroneComms, '/mutac/drone_comms', self.commsCallback, 100)

    def timerCallback(self):
        """At a certain rate it is checked the state of the drone along the mission.
        If an event is detected, it is published"""
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
            self.askReplan()
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

    def trajectoryCallback(self, msg):
        """Callback for the trajectory topic. It is used to set the waypoints of the drone"""
        self.get_logger().debug("********************")
        for path in msg.paths:
            self.get_logger().debug("Path "+str(path.identifier.natural) + ": "+str(len(path.points)))
            self.get_logger().debug("********************")
            self.drone.setWaypoints(path)

        # self.get_logger().info("Max orientation: "+str(self.max_orientation)+", min orientation: "+str(self.min_orientation))
        # self.get_logger().info("Max angular velocity: "+str(self.max_angular_velocity)+", min angular velocity: "+str(self.min_angular_velocity))
        # self.get_logger().info("Max linear acceleration: "+str(self.max_linear_acceleration)+", min linear acceleration: "+str(self.min_linear_acceleration))

    def imuCallback(self, msg):
        """Callback for the IMU topic. It is used to search for non-sense values"""
        # Writes the IMU value in the file
        self.file.write(str(msg.header.stamp.sec)+":"+str(msg.header.stamp.nanosec)+";"+str(msg.angular_velocity.x)+","+str(msg.angular_velocity.y)+","+str(msg.angular_velocity.z)+";"+str(msg.linear_acceleration.x)+","+str(msg.linear_acceleration.y)+","+str(msg.linear_acceleration.z)+";"+str(msg.orientation.x)+","+str(msg.orientation.y)+","+str(msg.orientation.z)+","+str(msg.orientation.w)+"\n")

        # if msg.orientation.x > self.max_orientation[0]:
        #     self.max_orientation = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        # if msg.orientation.y > self.max_orientation[1]:
        #     self.max_orientation = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        # if msg.orientation.z > self.max_orientation[2]:
        #     self.max_orientation = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        # if msg.orientation.w > self.max_orientation[3]:
        #     self.max_orientation = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        # if msg.orientation.x < self.min_orientation[0]:
        #     self.min_orientation = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        # if msg.orientation.y < self.min_orientation[1]:
        #     self.min_orientation = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        # if msg.orientation.z < self.min_orientation[2]:
        #     self.min_orientation = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        # if msg.orientation.w < self.min_orientation[3]:
        #     self.min_orientation = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)

        # if msg.angular_velocity.x > self.max_angular_velocity_x[0]:
        #     self.max_angular_velocity_x = (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)
        # if msg.angular_velocity.y > self.max_angular_velocity_y[1]:
        #     self.max_angular_velocity_y = (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)
        # if msg.angular_velocity.z > self.max_angular_velocity_z[2]:
        #     self.max_angular_velocity_z = (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)
        # if msg.angular_velocity.x < self.min_angular_velocity_x[0]:
        #     self.min_angular_velocity_x = (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)
        # if msg.angular_velocity.y < self.min_angular_velocity_y[1]:
        #     self.min_angular_velocity_y = (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)
        # if msg.angular_velocity.z < self.min_angular_velocity_z[2]:
        #     self.min_angular_velocity_z = (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)

        # if msg.linear_acceleration.x > self.max_linear_acceleration_x[0]:
        #     self.max_linear_acceleration_x = (msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z)
        # if msg.linear_acceleration.y > self.max_linear_acceleration_y[1]:
        #     self.max_linear_acceleration_y = (msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z)
        # if msg.linear_acceleration.z > self.max_linear_acceleration_z[2]:
        #     self.max_linear_acceleration_z = (msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z)
        # if msg.linear_acceleration.x < self.min_linear_acceleration_x[0]:
        #     self.min_linear_acceleration_x = (msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z)
        # if msg.linear_acceleration.y < self.min_linear_acceleration_y[1]:
        #     self.min_linear_acceleration_y = (msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z)
        # if msg.linear_acceleration.z < self.min_linear_acceleration_z[2]:
        #     self.min_linear_acceleration_z = (msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z) 
        pass

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
        """Callback for the alarm topic. In Aerostack it is not used"""
        if msg.identifier.natural == self.id:
            if msg.alarm == Alarm.CAMERA_FAILURE:
                self.drone.camera = False
                self.askReplan()


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