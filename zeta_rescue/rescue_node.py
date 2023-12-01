"""
This module contains the RescueNode class. It is used to represent a robot which is trying to 
randomly go to a location in space and come back to the original location

Author: Walker Todd, Jesse Yao, Jack Posada, and Jack Milman
"""
import random
import numpy as np
import math

import time
import numpy as np
from jmu_ros2_util import map_utils, pid
from std_msgs.msg import Empty

from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from nav_msgs.msg import OccupancyGrid

from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

import rclpy
import rclpy.node
from rclpy.action.client import ActionClient
from rclpy.task import Future

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, PointStamped
import tf_transformations
from tf2_geometry_msgs import PoseStamped

from zeta_competition_interfaces.msg import Victim as VictimMsg

from ros2_aruco_interfaces.msg import ArucoMarkers

from sensor_msgs.msg import Image
import tf2_ros

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs  # Import is needed, even though not used explicitly
from zeta_rescue import transformer


def make_random_goal():
    x = random.uniform(-0.50, 0.50)
    y = random.uniform(-0.50, 0.50)
    theta = random.uniform(-np.pi, np.pi)
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

        self.declare_parameter('time_limit', 0)
        timeout = self.get_parameter(
            'time_limit').get_parameter_value().integer_value
        self.get_logger().info(f"Time limit: {timeout: d}")

        # self.create_subscription(OccupancyGrid, 'map',
        #                          self.map_callback)
        self.create_subscription(
            PoseWithCovarianceStamped, 'amcl_pose', self.pose_callback, 10)
        self.create_subscription(
            ArucoMarkers, 'aruco_markers', self.scanner_callback, 10)
        self.create_subscription(
            Empty, '/report_requested', self.report_requested_callback, 10)
        self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)

        self.taken_picture = False
        self.victim_publisher = self.create_publisher(VictimMsg, '/victim', 10)

        self.wandering = False
        self.victim_poses = []
        self.victim_messages = []
        self.victims_complete = []

        self.goal = None
        self.goal_future = Future()
        self.initial_pose = None
        self.current_pose = None

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
        self.going_home = False

        # self.map = None
        self.time_elapsed = 0
        self.timeout = timeout

        self.transform_to_map = None
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        self.going_to_victim = False
        self.victim_search_id = 0
        self.victim_found = False
        self.victim_count = 0
        self.scan_timer = 0

    def image_callback(self, msg):
        self.taken_picture = msg

    def report_requested_callback(self, msg):
        self.get_logger().info("REPORT REQUESTED")
        self.get_logger().info(
            f"Number of victims found: {len(self.victims_complete)}")
        for victim in self.victims_complete:
            self.victim_publisher.publish(victim)
        self.get_logger().info("REPORT COMPLETE")

    def get_future(self):
        return self.node_future

    """
    Starts the wandering process. Should only be used if the robot has not already started wandering.
    """

    def start_wandering(self):
        if not self.wandering:
            first_goal = make_random_goal()
            self.update_goal(first_goal)

    """
    Sets the goal of the robot to the new_goal, then tells the robot to navigate toward it by calling send_goal()
    """

    def update_goal(self, new_goal):
        self.goal = new_goal
        self.get_logger().info(
            f"Goal is: {self.goal.pose.pose.position.x: 0.2f}, {self.goal.pose.pose.position.y: 0.2f}")
        self.send_goal()

    """
    Tells the robot to navigate toward the goal using an action client
    """

    def send_goal(self):
        self.get_logger().info("WAITING FOR NAVIGATION SERVER...")
        self.ac.wait_for_server()
        self.get_logger().info("NAVIGATION SERVER AVAILABLE...")
        self.get_logger().info("SENDING GOAL TO NAVIGATION SERVER...")
        self.start_time = time.time()

        self.goal_future = self.ac.send_goal_async(self.goal)

    """
    This creates a new victim in the robot's world-knowledge and hopefully doesn't create repeats. Returns true if the victim was new.
    """

    def make_victim(self, pose):
        victim_pose = tf2_geometry_msgs.PoseStamped()
        victim_pose.header.frame_id = "camera_rgb_optical_frame"
        victim_pose.pose = pose
        try:
            trans_pose = self.buffer.transform(victim_pose, "map")
            victim_pose.pose = trans_pose.pose
            trans_point_stamped = PointStamped()
            trans_point_stamped.point = trans_pose.pose.position
            trans_point_stamped.header.frame_id = "camera_rgb_optical_frame"

            if not self.duplicate_victim(trans_pose.pose.position, self.victim_messages):
                self.get_logger().info(
                    f"NEW VICTIM FOUND AT: X = {victim_pose.pose.position.x: .02f}, Y = {victim_pose.pose.position.y: .02f}")
                victim_info = VictimMsg()
                victim_info.id = self.victim_count
                self.victim_count += 1
                victim_info.point = trans_point_stamped
                victim_info.description = "Ronald the Journalist"

                self.victim_poses.append(victim_pose)
                self.victim_messages.append(victim_info)
                self.get_logger().info(
                    f"Number of victims: {len(self.victim_messages): d}")
                return True
        except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            # # Idk if this whole method works yet, but its a start
            # self.get_logger().warn(str(e))
            pass
        return False

    """
    Checks if a victim is a duplicate, and returns true if it is not a duplicate
    """

    def duplicate_victim(self, new_point, existing_victims):
        threshold = 0.3
        for victim in existing_victims:
            distance = np.linalg.norm(
                np.array([victim.point.point.x, victim.point.point.y]) -
                np.array([new_point.x, new_point.y])
            )
            if distance < threshold:
                return True
        return False

    """
    Checks if there are more detected victims and, if so, navs toward them to get a picture.
    """

    def search_for_victims(self):
        i = self.victim_search_id
        if i < len(self.victim_poses):
            self.going_to_victim = True
            vic_pose = self.victim_poses[i]
            orient = vic_pose.pose.orientation
            orientation_list = np.array(
                [orient.x, orient.y, orient.z, orient.w])
            vic_orient = tf_transformations.euler_from_quaternion(
                orientation_list)
            # here we get (1, 0, 0) in the victim's coordinate frame, facing the victim, in the map's coordinate frame.
            goal = self.victim_transform(vic_pose, i)
            next_goal = create_nav_goal(
                goal[0], goal[1], vic_orient[2] + np.pi / 2)  # X, Y, Theta Z
            self.update_goal(next_goal)

    """
    Transform (1, 0, 0) in victim's coordinate frame into a usable pose in the map's coordinate frame.
    """

    def victim_transform(self, pose, id):
        tran = transformer.Transformer("map")
        posit = pose.pose.position

        # victim to map conversion
        F_map_to_victim = transformer.trans(
            posit.x, posit.y, 0) @ transformer.rot_z(0)  # yaw is euler[2]
        tran.add_transform("map", f"victim{id: d}", F_map_to_victim)

        one_ahead_of_victim = np.array([0.75, 0, 0])
        goal = tran.transform(f"victim{id: d}", "map", one_ahead_of_victim)
        return goal

    """
    Shoots a photograph of the victim, sets going_to_victim to false, and adds the victim to the victims_complete list
    """

    def shoot_photo(self):
        self.going_to_victim = False
        self.get_logger().info(
            f"Len victim_messages: {len(self.victim_messages): d}")
        self.get_logger().info(f"Search ID: {self.victim_search_id: d}")
        victim = self.victim_messages[self.victim_search_id]
        victim.image = self.taken_picture
        self.victims_complete.append(victim)
        self.victim_search_id += 1
        self.get_logger().info("Picture shot")

    """
    Logic for determining which goal to go to next.
    """

    def do_next_navigation(self):
        victims_remaining = len(self.victims_complete) < len(self.victim_messages)
        self.get_logger().info(
            f"NUMBER OF VICTIMS LEFT: {len(self.victim_messages) - len(self.victims_complete): d}")
        time_remaining = self.time_elapsed < (self.timeout - 20)
        if victims_remaining:
            self.get_logger().info("GOING TO NEXT KNOWN VICTIM")
            self.search_for_victims()
        elif time_remaining:
            self.get_logger().info("RESCUEBOT CONTINUING SEARCH IN NEW POSITION")
            new_goal = make_random_goal()
            self.update_goal(new_goal)
        else:
            # hacky way to end the movement, fix later
            self.completed_navs = 1000000000000000

    def scanner_callback(self, aruco_msg):
        if self.scan_timer >= 7:
            self.scan_timer = 0
            for pose in aruco_msg.poses:
                if self.make_victim(pose):
                    # We are gonna want to move to the victim location
                    # Take a picture then appending the victim infromation to the victim messages array
                    if not self.going_to_victim:
                        self.goal_future.result().cancel_goal_async()
                        self.going_to_victim = True
                        self.search_for_victims()
                        self.get_logger().info(f"GOING TO NEW VICTIM")

    """
    This method is keeping track of the overall status of the node's search
    """

    def wander_callback(self):
        if self.goal_future.done():
            if self.completed_navs >= self.max_iterations and not self.going_home:
                self.get_logger().info("RESCUEBOT GOING HOME!")
                self.goal_future.result().cancel_goal_async()
                initial_x = self.initial_pose.pose.pose.position.x
                initial_y = self.initial_pose.pose.pose.position.y
                initial_orient = self.initial_pose.pose.pose.orientation
                home = create_nav_goal_quat(
                    initial_x, initial_y, initial_orient)
                self.update_goal(home)
                self.going_home = True
            # If an individual navigation goal is complete.
            elif self.navigation_complete:
                self.navigation_complete = False
                if self.going_to_victim:
                    self.shoot_photo()
                self.do_next_navigation()

        elif not self.wandering:
            self.get_logger().info("RESCUEBOT ROLLING OUT")
            self.start_wandering()
        self.time_elapsed += 1
        # self.get_logger().info(f"TIME ELAPSED: {self.time_elapsed: d}")

    """
    Periodically check in on the progress of navigation.
    """

    def timer_callback(self):
        if not self.goal_future.done():
            self.get_logger().info("NAVIGATION GOAL NOT YET ACCEPTED")

        elif self.cancel_future is not None:  # We've cancelled and are waiting for ack.
            if self.cancel_future.done():
                self.get_logger().info("SERVER HAS ACKNOWLEDGED CANCELLATION")
                self.node_future.set_result(False)

        else:
            if not self.navigation_complete:
                if self.goal_future.result().status == GoalStatus.STATUS_SUCCEEDED:
                    self.get_logger().info("NAVIGATION SERVER REPORTS SUCCESS!")
                    self.navigation_complete = True

                elif self.goal_future.result().status == GoalStatus.STATUS_ABORTED:
                    self.get_logger().info("NAVIGATION SERVER HAS ABORTED.")
                    self.node_future.set_result(False)
                    self.navigation_complete = True

            elif self.going_home:
                if self.goal_future.result().status == GoalStatus.STATUS_SUCCEEDED:
                    self.node_future.set_result(True)

            # elif time.time() - self.start_time > self.timeout:
            #     self.get_logger().info("TAKING TOO LONG. CANCELLING GOAL!")
            #     self.cancel_future = self.goal_future.result().cancel_goal_async()
        self.scan_timer += 1

    def pose_callback(self, amcl_pose):
        self.current_pose = amcl_pose
        # x = amcl_pose.pose.pose.position.x
        # y = amcl_pose.pose.pose.position.y
        # self.get_logger().info(f"Robot Position: {x: .02f}, {y: .02f}")
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

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
