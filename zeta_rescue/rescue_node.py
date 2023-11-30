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

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, PointStamped, PoseStamped
import tf_transformations


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
        self.create_subscription(ArucoMarkers, 'aruco_markers', self.scanner_callback, 10)
        self.create_subscription(
            Empty, '/report_requested', self.report_requested_callback, 10)
        self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        
        self.taken_picture = False
        self.victim_publisher = self.create_publisher(VictimMsg, '/victim', 10)

        self.wandering = False
        self.victim_locations = []
        self.victims_shot = [] # with a camera, of course

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

        self.victim_transformed_pose = None
        self.going_to_victim = False
        self.victim_search_id = 0
        self.victim_found = False
        self.victim_count= 0

    def image_callback(self, msg):
        self.taken_picture = msg

    def report_requested_callback(self, msg):
        self.get_logger().info("REPORT REQUESTED")
        self.get_logger().info(
            f"Number of victims found: {len(self.victim_locations)}")
        for victim in self.victim_locations:
            self.victim_publisher.publish(victim)
        self.get_logger().info("REPORT COMPLETE")

    def get_future(self):
        return self.node_future

    # Starts the wandering process. Should only be used if the robot has not already started wandering.
    def start_wandering(self):
        if not self.wandering:
            first_goal = make_random_goal()
            self.update_goal(first_goal)

    # Sets the goal of the robot to the new_goal, then tells the robot to navigate toward it by calling send_goal()
    def update_goal(self, new_goal):
        self.goal = new_goal
        self.get_logger().info(
            f"Goal is: {self.goal.pose.pose.position.x: 0.2f}, {self.goal.pose.pose.position.y: 0.2f}")
        self.send_goal()

    # Tells the robot to navigate toward the goal using an action client
    def send_goal(self):
        self.get_logger().info("WAITING FOR NAVIGATION SERVER...")
        self.ac.wait_for_server()
        self.get_logger().info("NAVIGATION SERVER AVAILABLE...")
        self.get_logger().info("SENDING GOAL TO NAVIGATION SERVER...")
        self.start_time = time.time()

        self.goal_future = self.ac.send_goal_async(self.goal)

    def scanner_callback(self, aruco_msg):
        # self.get_logger().info("Did you find something?")
        # self.get_logger().info("Poses:")

        for pose in aruco_msg.poses:

            victim_pose = PoseStamped()
            self.get_logger().info(f"X: {victim_pose.pose.position.x}")
            self.get_logger().info(f"Y: {victim_pose.pose.position.y}")
            self.get_logger().info(f"Theta: {tf_transformations.euler_from_quaternion(victim_pose.pose.orientation)}")
            victim_pose.header.frame_id = "camera_rgb_optical_frame"
            victim_pose.pose = pose

            try:
                self.get_logger().info(f"Help me")
                transformed_pose = self.buffer.transform(victim_pose, "map")

                victim_info = VictimMsg()
                victim_info.id =  self.victim_count
                self.victim_count += 1
                victim_info.point = transformed_pose.pose.position
                victim_info.description = "Ronald the Journalist"

                if not self.is_duplicate_victim(transformed_pose.pose.position, self.victim_locations):
                    self.victim_locations.append(victim_info)
                    self.get_logger().info(f"YOU FOUND A VICTIM BRO GO TAKE A PICTURE")
                    self.get_logger().info(
                        f"Transformed X = {transformed_pose.pose.position.x:.2f}")
                    self.get_logger().info(
                        f"Transformed Y = {transformed_pose.pose.position.y:.2f}")

                    self.goal_future.result().cancel_goal_async()

                    # updated_victim_pose = pose
                    # updated_victim_pose.position.x = transformed_point.point.x
                    # updated_victim_pose.position.y = transformed_point.point.y
                    # updated_victim_pose.position.z = transformed_point.point.z
                    # updated_victim_pose.orientation = pose.orientation
                    # self.get_logger().info("THIS IS BEFORE THE VICTIM TRANSLATION")
                    # new_matrix = self.victim_trans(updated_victim_pose)
                    # # We are gonna want to move to the victim location
                    # # Take a picture then appending the victim infromation to the victim locations array
                    # home = create_nav_goal(new_matrix[0], new_matrix[1], new_matrix[2])
                    victim_goal = create_nav_goal_quat(transformed_pose.pose.position.x, transformed_pose.pose.position.y, transformed_pose.pose.orientation)
                    if not self.going_to_victim:
                        self.victim_search_id = victim_info.id
                        self.update_goal(victim_goal)
                        self.get_logger().info(f"GOING TO VICTIM")
                        self.going_to_victim = True
                    # self.get_logger().info(f"HI ARE YOU GETTING HERE SMILE") 
                    # # wait for the goal to be compelete

                # if self.goal_future.result().status == GoalStatus.STATUS_SUCCEEDED:
                #     victim = VictimMsg()

                #     victim.id =  self.victim_count 
                #     self.victim_count += 1
                #     victim.point = transformed_point
                #     victim.description = "Jerry the Journalist"
                #     victim.image = self.taken_picture

                #     self.victim_locations.append(victim)

            except Exception as e:
                # Idk if this whole method works yet, but its a start
                self.get_logger().warn(str(e))

    def is_duplicate_victim(self, new_point, existing_victims):
        threshold = 0.3
        for victim in existing_victims:
            distance = np.linalg.norm(
                np.array([victim.point.point.x, victim.point.point.y]) -
                np.array([new_point.point.x, new_point.point.y])
            )
            if distance < threshold:
                return True
        return False

    """
    Transform from (0, 0, 0) in map frame to victim frame then a second transformation
    from the victim to 1m in front of victim
    """
    def victim_trans(self, pose):
        tran = transformer.Transformer()

        # Create the quaternion then convert it to eulers
        quat = np.array(
            [pose.orientation.x, pose.orientation.y,
                pose.orientation.z, pose.orientation.w]
        )
        euler = tf_transformations.euler_from_quaternion(quat)

        # map to victim
        F_m_v = transformer.trans(
            pose.position.x, pose.position.y, pose.position.z) @ transformer.rot_z(euler[2]) # yaw is euler[2]
        tran.add_transform("map", "victim", F_m_v)

    # victim to front of victim
        F_v_f = transformer.trans(pose.position.x + 1, pose.position.y,
                                  pose.position.z) @ transformer.rot_z(np.pi)
        tran.add_transform("victim", "front", F_v_f)

        return tran.transform('victim', 'front', np.array([0, 0, 0]))
    
    """
    Checks if there are more detected victims and, if so, navs toward them to get a picture.
    """
    def search_for_victims(self):
        self.victim_search_id += 1
        i = self.victim_search_id
        if i < len(self.victim_locations):
            next_goal = self.victim_locations[i]

        pass


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
                    self.going_to_victim = False
                    # THEN WE TAKE A PICTURE
                    # [INSERT CODE HERE]
                    # 
                    victim = self.victim_locations[self.victim_search_id]
                    victim.image = self.taken_picture
                    self.victims_shot.append(victim)
                    self.search_for_victims()
                elif self.time_elapsed < self.timeout:
                    self.get_logger().info("RESCUEBOT CONTINUING SEARCH IN NEW POSITION")
                    new_goal = make_random_goal()
                    self.update_goal(new_goal)
        elif not self.wandering:
            self.get_logger().info("RESCUEBOT ROLLING OUT")
            self.start_wandering()
        self.time_elapsed += 1

    def timer_callback(self):
        """Periodically check in on the progress of navigation."""

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

    def pose_callback(self, amcl_pose):
        self.current_pose = amcl_pose
        x = amcl_pose.pose.pose.position.x
        y = amcl_pose.pose.pose.position.y
        self.get_logger().info(f"Position: {x: .02f}, {y: .02f}")
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
