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

from geometry_msgs.msg import PoseWithCovarianceStamped

import tf_transformations
from zeta_competition_interfaces.msg import Victim

from ros2_aruco_interfaces.msg import ArucoMarkers

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

    def __init__(self, goal, timeout):
        super().__init__('rescue_node')

        # This QOS Setting is used for topics where the messages
        # should continue to be available indefinitely once they are
        # published. Maps fall into this category.  They typically
        # don't change, so it makes sense to publish them once.
        latching_qos = QoSProfile(depth=1,
                                  durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        self.create_subscription(OccupancyGrid, 'map',
                                 self.map_callback,
                                 qos_profile=latching_qos)
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self.test_callback, 10)
        # self.create_publisher(Victim, 'victim_pose', self.timer_callback, 10)
        self.create_subscription(ArucoMarkers, 'aruco_markers', self.scanner_callback, 10)
        self.goal = goal
        self.initial_pose = None

        # Create the action client.
        self.ac = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # Used if we decide to cancel a goal request. None indicates
        # we haven't canceled.
        self.cancel_future = None
        
        self.map = None
        self.timeout = timeout
    
    def update_goal(self, new_goal):
        self.goal = new_goal
        self.ac = ActionClient(self, NavigateToPose, '/navigate_to_pose')
    
    def send_goal(self):
        self.get_logger().info("WAITING FOR NAVIGATION SERVER...")
        self.ac.wait_for_server()
        self.get_logger().info("NAVIGATION SERVER AVAILABLE...")
        self.get_logger().info("SENDING GOAL TO NAVIGATION SERVER...")
        self.start_time = time.time()

        self.goal_future = self.ac.send_goal_async(self.goal)

        self.create_timer(.1, self.timer_callback)

        # This will be used by rclpy to know when this node has
        # finished its work.
        self.future_event = Future()

        return self.future_event
    
    def scanner_callback(self, aruco_msg):
        self.get_logger().info("Did you find something?")
        for marker in aruco_msg.markers:
            if marker.id == 0:
                self.get_logger().info("Found a victim!")
                victim = Victim()
                victim.header.frame_id = "map"
                victim.pose.pose.position.x = marker.pose.position.x
                victim.pose.pose.position.y = marker.pose.position.y
                victim.pose.pose.position.z = marker.pose.position.z
                victim.pose.pose.orientation.x = marker.pose.orientation.x
                victim.pose.pose.orientation.y = marker.pose.orientation.y
                victim.pose.pose.orientation.z = marker.pose.orientation.z
                victim.pose.pose.orientation.w = marker.pose.orientation.w
                victim.pose.covariance = marker.pose.covariance
                victim.id = marker.id
                victim.distance = marker.distance
                victim.confidence = marker.confidence
                self.victim_pub.publish(victim)
                self.get_logger().info("Published victim!")
            else:
                self.get_logger().info("Found a marker, but not a victim.")

                
        
    def map_callback(self, map_msg):
        """Process the map message.

        This doesn't really do anything useful, it is purely intended
        as an illustration of the Map class.

        """
        if self.map is None:  # No need to do this every time map is published.

            self.map = map_utils.Map(map_msg)

            # Use numpy to calculate some statistics about the map:
            total_cells = self.map.width * self.map.height
            pct_occupied = np.count_nonzero(self.map.grid == 100) / total_cells * 100
            pct_unknown = np.count_nonzero(self.map.grid == -1) / total_cells * 100
            pct_free = np.count_nonzero(self.map.grid == 0) / total_cells * 100
            map_str = "Map Statistics: occupied: {:.1f}% free: {:.1f}% unknown: {:.1f}%"
            self.get_logger().info(map_str.format(pct_occupied, pct_free, pct_unknown))

            # Here is how to access map cells to see if they are free:
            x = self.goal.pose.pose.position.x
            y = self.goal.pose.pose.position.y
            val = self.map.get_cell(x, y)
            if val == 100:
                free = "occupied"
            elif val == 0:
                free = "free"
            else:
                free = "unknown"
            self.get_logger().info(f"HEY! Map position ({x:.2f}, {y:.2f}) is {free}")

    def timer_callback(self):
        """Periodically check in on the progress of navigation."""

        if not self.goal_future.done():
            self.get_logger().info("NAVIGATION GOAL NOT YET ACCEPTED")

        elif self.cancel_future is not None:  # We've cancelled and are waiting for ack.
            if self.cancel_future.done():
                self.get_logger().info("SERVER HAS ACKNOWLEDGED CANCELLATION")
                self.ac.destroy()
                self.future_event.set_result(False)
        else:

            if self.goal_future.result().status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info("NAVIGATION SERVER REPORTS SUCCESS. EXITING!")
                self.ac.destroy()
                self.future_event.set_result(True)

            if self.goal_future.result().status == GoalStatus.STATUS_ABORTED:
                self.get_logger().info("NAVIGATION SERVER HAS ABORTED. EXITING!")
                self.ac.destroy()
                self.future_event.set_result(False)

            elif time.time() - self.start_time > self.timeout:
                self.get_logger().info("TAKING TOO LONG. CANCELLING GOAL!")
                self.cancel_future = self.goal_future.result().cancel_goal_async()
    
    def test_callback(self, amcl_pose):
        if self.initial_pose is None:
            self.initial_pose = amcl_pose
        self.get_logger().info("I AM ALIVE")


def main():
    rclpy.init()
    x = random.uniform (-2.0, 2.0)
    y = random.uniform (-2.0, 2.0)
    theta = random.uniform (-np.pi, np.pi)
    timeout = float('inf')
    first_goal = create_nav_goal(x, y, theta)
    node = RescueNode(first_goal, timeout)
    # Arbitrary wandering number for testing purposes, can change later.
    i = 0
    while i < 2:
        x = random.uniform (-2.0, 2.0)
        y = random.uniform (-2.0, 2.0)
        theta = random.uniform (-np.pi, np.pi)
        new_goal = create_nav_goal(x, y, theta)
        node.update_goal(new_goal)
        future = node.send_goal()
        rclpy.spin_until_future_complete(node, future)
        node.get_logger().info("Node's future: " + str(future.result()))
        i += 1
    # Navigates back to the initial position and then exits cleanly we hope.
    initial_x = node.initial_pose.pose.pose.position.x
    initial_y = node.initial_pose.pose.pose.position.y
    initial_orient = node.initial_pose.pose.pose.orientation
    go_back = create_nav_goal_quat(initial_x, initial_y, initial_orient)
    node.update_goal(go_back)
    future = node.send_goal()
    rclpy.spin_until_future_complete(node, future)
    node.get_logger().info("Node's future: " + str(future.result()))



    node.destroy_node()
    rclpy.shutdown()    

if __name__ == '__main__':
    main()
