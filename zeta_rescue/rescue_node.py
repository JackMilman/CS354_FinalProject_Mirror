"""
This module contains the RescueNode class. It is used to represent a robot which is trying to 
randomly go to a location in space and come back to the original location

Author: Walker Todd, Jesse Yao, Jack Posada, and Jack Milman
"""
import random
import numpy as np

import time
import numpy as np
from jmu_ros2_util import map_utils

from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from nav_msgs.msg import OccupancyGrid

from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

import rclpy
import rclpy.node
from rclpy.action.client import ActionClient
from rclpy.task import Future

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray

import tf_transformations
from zeta_competition_interfaces.msg import Victim

from ros2_aruco_interfaces.msg import ArucoMarkers

def make_random_goal():
    x = random.uniform (-2.0, 2.0)
    y = random.uniform (-2.0, 2.0)
    theta = random.uniform (-np.pi, np.pi)
    random_goal = create_nav_goal(x, y, theta)
    return random_goal

def create_nav_goal(x, y, theta):
    goal = NavigateToPose.Goal()

    goal.pose.header.frame_id = 'map'
    goal.pose.pose.position.x = x
    goal.pose.pose.position.y = y

    # We need to convert theta to a quaternion....
    quaternion = tf_transformations.quaternion_from_euler(0, 0, theta, 'rxyz')
    goal.pose.pose.orientation.x = quaternion[0]
    goal.pose.pose.orientation.y = quaternion[1]
    goal.pose.pose.orientation.z = quaternion[2]
    goal.pose.pose.orientation.w = quaternion[3]
    return goal

def create_nav_goal_quat(x, y, quaternion):
    goal = NavigateToPose.Goal()

    goal.pose.header.frame_id = 'map'
    goal.pose.pose.position.x = x
    goal.pose.pose.position.y = y

    goal.pose.pose.orientation = quaternion
    return goal

class RescueNode(rclpy.node.Node):

    def __init__(self, timeout, iterations):
        super().__init__('rescue_node')

        # This QOS Setting is used for topics where the messages
        # should continue to be available indefinitely once they are
        # published. Maps fall into this category.  They typically
        # don't change, so it makes sense to publish them once.
        latching_qos = QoSProfile(depth=1,
                                  durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        # self.create_subscription(OccupancyGrid, 'map',
        #                          self.map_callback,
        #                          qos_profile=latching_qos)
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self.init_pose_callback, 10)
        # self.create_publisher(Victim, 'victim_pose', self.timer_callback, 10)
        # self.create_subscription(PoseArray, 'aruco_poses', self.scanner_callback, 10)

        self.wandering = False

        # self.goal = goal
        self.goal = None
        self.goal_future = Future()
        self.initial_pose = None

        # Create the action client.
        self.ac = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.completed_navs = 0
        self.navigation_complete = False
        self.max_iterations = iterations
        self.create_timer(.1, self.timer_callback)
        self.create_timer(1, self.wander_callback)

        # Used if we decide to cancel a goal request. None indicates
        # we haven't canceled.
        self.cancel_future = None
        # Used to keep track of the entire node's progress and to exit when we want to.
        self.node_future = Future()
        
        # self.map = None
        self.timeout = timeout

    def get_future(self):
        return self.node_future
    
    def start_wandering(self):
        if not self.wandering:
            first_goal = make_random_goal()
            self.update_goal(first_goal)
    
    def update_goal(self, new_goal):
        self.goal = new_goal
        self.send_goal()
    
    def send_goal(self):
        self.get_logger().info("WAITING FOR NAVIGATION SERVER...")
        self.ac.wait_for_server()
        self.get_logger().info("NAVIGATION SERVER AVAILABLE...")
        self.get_logger().info("SENDING GOAL TO NAVIGATION SERVER...")
        self.start_time = time.time()

        self.goal_future = self.ac.send_goal_async(self.goal)
    
    def scanner_callback(self, aruco_msg):
        self.get_logger().info("Did you find something?")
        self.get_logger().info("Poses:")
        for pose in aruco_msg.poses:
            self.get_logger().info(f"X = {pose.position.x:.2f}")
            self.get_logger().info(f"Y = {pose.position.y:.2f}")

    """
    This method is keeping track of the overall status of the node's search
    """
    def wander_callback(self):
        if self.node_future.done():
            self.get_logger().info("RESCUEBOT RETURNING HOME!")
        elif self.goal_future.done():
            if self.completed_navs >= self.max_iterations:
                self.get_logger().info("MAXIMUM ITERATIONS REACHED, EXITING!")
                self.node_future.set_result(True)
            elif self.navigation_complete:
                self.navigation_complete = False
                self.completed_navs += 1
                self.get_logger().info(f"{self.completed_navs}")
                self.get_logger().info("RESCUEBOT CONTINUING SEARCH IN NEW POSITION")
                x = random.uniform (-2.0, 2.0)
                y = random.uniform (-2.0, 2.0)
                theta = random.uniform (-np.pi, np.pi)
                new_goal = create_nav_goal(x, y, theta)
                self.update_goal(new_goal)
        elif not self.wandering:
            self.start_wandering()

    def timer_callback(self):
        """Periodically check in on the progress of navigation."""

        if self.completed_navs >= self.max_iterations:
            self.get_logger().info("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA")
            self.cancel_future = self.goal_future.result().cancel_goal_async()

        if not self.goal_future.done():
            self.get_logger().info("NAVIGATION GOAL NOT YET ACCEPTED")

        elif self.cancel_future is not None:  # We've cancelled and are waiting for ack.
            if self.cancel_future.done():
                self.get_logger().info("SERVER HAS ACKNOWLEDGED CANCELLATION")
                # self.ac.destroy()
                self.node_future.set_result(False)
                # self.future_event.set_result(False)
        else:
            if not self.navigation_complete:
                if self.goal_future.result().status == GoalStatus.STATUS_SUCCEEDED:
                    self.get_logger().info("NAVIGATION SERVER REPORTS SUCCESS!")
                    self.navigation_complete = True
                    # self.ac.destroy()
                    # self.future_event.set_result(True)
                elif self.goal_future.result().status == GoalStatus.STATUS_ABORTED:
                    self.get_logger().info("NAVIGATION SERVER HAS ABORTED.")
                    self.node_future.set_result(False)
                    self.navigation_complete = True
                    # self.ac.destroy()
                    # self.future_event.set_result(False)

            # elif time.time() - self.start_time > self.timeout:
            #     self.get_logger().info("TAKING TOO LONG. CANCELLING GOAL!")
            #     self.cancel_future = self.goal_future.result().cancel_goal_async()
        
    
    def init_pose_callback(self, amcl_pose):
        if self.initial_pose is None:
            self.initial_pose = amcl_pose
            self.get_logger().info("Initial pose set")


def main():
    rclpy.init()

    timeout = float('inf')
    # test parameter for determining how long to randomly wander, change later
    iterations = 2
    node = RescueNode(timeout, iterations)
    node_future = node.get_future()
    rclpy.spin_until_future_complete(node, node_future)

    # Navigates back to the initial position and then exits cleanly we hope.
    initial_x = node.initial_pose.pose.pose.position.x
    initial_y = node.initial_pose.pose.pose.position.y
    initial_orient = node.initial_pose.pose.pose.orientation
    go_back = create_nav_goal_quat(initial_x, initial_y, initial_orient)
    node.update_goal(go_back)
    node.node_future = Future()
    node_future = node.get_future()
    rclpy.spin_until_future_complete(node, node_future)
    node.get_logger().info("Node's future: " + str(node_future.result()))



    node.destroy_node()
    rclpy.shutdown()    

if __name__ == '__main__':
    main()
