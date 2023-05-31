import rclpy
from rclpy.node import Node

from mutac_msgs.msg import Plan
from mutac_msgs.srv import Replan, UpdatePlan

from std_msgs.msg import Empty

import time


class Replanner(Node):
    def __init__(self):
        """Replanning manager node initialization. Services are mainly used to make the communication with the 
        monitor nodes more reliable."""
        super().__init__('Replanning_manager')

        # Publisher
        self.replan_pub = self.create_publisher(Empty, '/planner/replanning/request_wps', 100)

        # Services
        self.ask_replan_srv = self.create_service(Replan, '/planner/replanning/ask_replan', self.replanCallback)
        self.provide_wp_srv = self.create_service(Replan, '/planner/replanning/provide_wps', self.pathCallback)

        # Client
        self.replan_client = self.create_client(UpdatePlan, '/planner/update_plan')

        self.plan = Plan()

        self.timer_period = 1 # seconds
        self.timer = None


    def timerCallback(self):
        """Timer callback. It is used when a replan is requested and the path 
        collection time is over."""
        self.get_logger().debug("Time: "+str(time.time()))
        if len(self.plan.paths) > 1:
            srv = UpdatePlan.Request()
            srv.plan = self.plan
            self.get_logger().info("Plan: "+str(len(self.plan.paths)))
            while not self.replan_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn('service not available, waiting again...')
            
            self.replan_client.call_async(srv) # Calls the planner service that makes the replan
        else:
            self.get_logger().info("Not enough paths to replan")
        self.plan = Plan()
        self.timer.cancel()
    
    def replanCallback(self, request, response):
        """Replan callback. It is called when a replan is requested."""
        self.get_logger().info("********************")
        self.get_logger().info("Replan received")
        self.get_logger().info("********************")
        self.plan.paths.append(request.path)
        self.get_logger().info("Plan: "+str(len(self.plan.paths)))
        self.replan_pub.publish(Empty())

        # Timer to wait for the paths
        if self.timer is None:
            self.timer = self.create_timer(self.timer_period, self.timerCallback)
        else:
            self.timer.reset()
        self.get_logger().debug("Time: "+str(time.time()))
        return response

    def pathCallback(self, request, response):
        """Path callback. It is called when a path is provided by a drone monitor."""
        self.get_logger().debug("Received path: "+str(time.time()))
        self.plan.paths.append(request.path)
        return response


def main(args=None):
    rclpy.init(args=args)

    replanner = Replanner()
    rclpy.spin(replanner)

if __name__ == '__main__':
    main()