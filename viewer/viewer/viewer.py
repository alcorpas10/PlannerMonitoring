import rclpy
from rclpy import qos
from rclpy.node import Node

from mutac_msgs.msg import Alarm, State, Plan, Identifier, LabeledPath

from geometry_msgs.msg import Point, PoseStamped, Pose
from nav_msgs.msg import Path

import math


class Viewer(Node):
    def __init__(self):
        super().__init__("viewer", allow_undeclared_parameters=True, 
                         automatically_declare_parameters_from_overrides=True)
        
        self.n_drones = self.get_parameter('n_drones').value

        self.get_logger().info("Drones: " + str(self.n_drones))
        self.covered_points = [[] for _ in range(self.n_drones)]
        self.left_points = [[] for _ in range(self.n_drones)]

        self.pose_subs = []
        self.left_pubs = []
        self.covered_pubs = []

        self.trj_sub = self.create_subscription(Plan, '/mutac/planned_paths', self.trajectoryCallback, qos.QoSProfile(reliability=qos.ReliabilityPolicy.RELIABLE, depth=10))
        self.covered_sub = self.create_subscription(Identifier, '/mutac/covered_points', self.coveredCallback, qos.QoSProfile(reliability=qos.ReliabilityPolicy.RELIABLE, depth=100))
        # self.events_sub = self.create_subscription(State, '/mutac/drone_events', self.eventCallback, qos.QoSProfile(reliability=qos.ReliabilityPolicy.RELIABLE, depth=100))

        for i in range(self.n_drones):
            uav_name = "/drone_sim_"+str(i)
            # uav_name = "cf"+str(i)
            publisher = "/path" + str(i)
            
            self.pose_subs.append(self.create_subscription(PoseStamped, uav_name+'/self_localization/pose', 
                                                           lambda msg, id=i: self.positionCallback(msg.pose, id), qos.qos_profile_sensor_data))
            self.left_pubs.append(self.create_publisher(Path, publisher + "_left", 10))
            self.covered_pubs.append(self.create_publisher(Path, publisher + "_covered", 10))
        
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.timerCallback)


    def trajectoryCallback(self, msg):
        self.get_logger().info("Got plan")
        self.left_points = [[] for _ in range(self.n_drones)]
        received_path = []

        for path in msg.paths:
            path_id = path.identifier.natural
            received_path.append(path_id)
            for p in path.points[1:]:
                self.left_points[path_id].append([p.point.x, p.point.y, p.point.z, p.label])
            self.covered_points[path_id].append([path.points[0].point.x, path.points[0].point.y, path.points[0].point.z, path.points[0].label])
        for i in range(self.n_drones):
            self.get_logger().info("Drone " + str(i) + " has " + str(len(self.left_points[i])) + " left points")
            if i not in received_path:
                self.covered_points[i].append(self.pose)

    def positionCallback(self, msg, id):
        """Saves the new position of the drone and appends it at the end of the covered points
        list and at the beginning of the left points list. Before appending the new position,
        the old one is removed"""
        self.pose = (msg.position.x, msg.position.y, msg.position.z)
        if len(self.left_points[id]) > 0:
            self.left_points[id].pop(0)
        if len(self.covered_points[id]) > 0:
            self.covered_points[id].pop()
        if len(self.left_points[id]) > 0 and len(self.covered_points[id]) > 0:
            pos_in_trj = self.calculateProjection(self.pose, self.covered_points[id][-1], self.left_points[id][0])
            self.covered_points[id].append(pos_in_trj)
        else:
            self.covered_points[id].append(self.pose)
        if len(self.left_points[id]) > 1:
            self.left_points[id].insert(0, self.pose)

    def calculateProjection(self, pos, wp1, wp2):
        """Returns the projection of a point in a line"""
        AM = (pos[0] - wp1[0], pos[1] - wp1[1], pos[2] - wp1[2])
        u = (wp2[0] - wp1[0], wp2[1] - wp1[1], wp2[2] - wp1[2])

        scalar = (AM[0] * u[0] + AM[1] * u[1] + AM[2] * u[2]) / (math.sqrt(u[0]**2 + u[1]**2 + u[2]**2)**2)
        
        return (wp1[0] + scalar * u[0], wp1[1] + scalar * u[1], wp1[2] + scalar * u[2])

    def coveredCallback(self, msg):
        drone_id = msg.natural
        # Append the second point of the left points list to the second to last position of the covered points list
        if len(self.left_points[drone_id]) > 1 and len(self.covered_points[drone_id]) >= 1:
            self.covered_points[drone_id].insert(-1, self.left_points[drone_id][1])
            self.left_points[drone_id].pop(1)
            self.get_logger().info("Length: "+str(len(self.left_points[drone_id])))

    def timerCallback(self):
        """Publishes the covered and left points lists"""
        for i in range(self.n_drones):
            poses_list_left = []
            for p in self.left_points[i]:
                poses_list_left.append(PoseStamped(pose=Pose(position=Point(
                    x=p[0],
                    y=p[1],
                    z=p[2]
                ))))

            path_msg = Path()
            path_msg.header.frame_id = 'earth'
            path_msg.poses = poses_list_left
            self.left_pubs[i].publish(path_msg)

            poses_list_cov = []
            for p in self.covered_points[i]:
                poses_list_cov.append(PoseStamped(pose=Pose(position=Point(
                    x=p[0],
                    y=p[1],
                    z=p[2]
                ))))

            path_msg = Path()
            path_msg.header.frame_id = 'earth'
            path_msg.poses = poses_list_cov
            self.covered_pubs[i].publish(path_msg)


def main(args=None):
    rclpy.init(args=args)
    viewer = Viewer()
    rclpy.spin(viewer)

if __name__ == '__main__':
    main()