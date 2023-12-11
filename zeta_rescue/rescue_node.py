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
from jmu_ros2_util.map_utils import Map
from std_msgs.msg import Empty

from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid

from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

import rclpy
import rclpy.node
from rclpy.action.client import ActionClient
from rclpy.task import Future

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, PointStamped, Pose, Point
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
from zeta_rescue import victim_namer as vn

class RescueNode(rclpy.node.Node):

    def __init__(self, iterations):
        super().__init__('rescue_node')

        self.declare_parameter('time_limit', 0)
        timeout = self.get_parameter(
            'time_limit').get_parameter_value().integer_value
        self.get_logger().info(f"Time limit: {timeout: d}")

        self.initial_pose = None
        self.current_pose = None

        latching_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            OccupancyGrid, 'map', self.map_callback, qos_profile=latching_qos)
        # self.create_subscription(
        #     OccupancyGrid, 'costmap_updates', self.costmap_callback, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, 'initialpose', self.initial_pose_callback, 10)
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

        self.goal : NavigateToPose.Goal = NavigateToPose.Goal()
        self.goal_future = Future()

        # Create the action client.
        self.ac = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.completed_navs = 0
        self.navigation_complete = False
        self.max_iterations = iterations
        self.create_timer(.1, self.navigation_callback)
        self.create_timer(1, self.wander_callback)

        # Used if we decide to cancel a goal request. None indicates
        # we haven't canceled.
        self.cancel_future = None
        # Used to keep track of the entire node's progress and to exit when we want to.
        self.node_future = Future()
        self.going_home = False

        self.time_elapsed = 0
        self.timeout = timeout - 20

        self.transform_to_map = None
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        self.going_to_victim = False
        self.victim_search_id = 0
        self.victim_found = False
        self.victim_count = 0
        self.scan_timer = 0

        self.map : Map = None
        self.explored_goals = [] # List of Points
        self.near_check_iterations = 0
        self.near_check_threshold = 50
        self.random_iterations_threshold = 10000
        self.close_threshold = 2.0

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

    def start_wandering(self):
        """
        Starts the wandering process. Should only be used if the robot has not already started wandering.
        """
        if not self.wandering:
            first_goal = self.make_new_map_goal()
            self.update_goal(first_goal)
            self.wandering = True

    def update_goal(self, new_goal: NavigateToPose.Goal):
        """
        Sets the goal of the robot to the new_goal, then tells the robot to navigate toward it by calling send_goal()
        """
        self.goal = new_goal

        orient = self.goal.pose.pose.orientation
        orientation_list = np.array([orient.x, orient.y, orient.z, orient.w])
        goal_theta = tf_transformations.euler_from_quaternion(orientation_list)

        self.get_logger().info(
            f"Goal is: {self.goal.pose.pose.position.x: 0.2f}, {self.goal.pose.pose.position.y: 0.2f}, Theta = {goal_theta[2]: .02f}")
        self.send_goal()

    def send_goal(self):
        """
        Tells the robot to navigate toward the goal using an action client
        """

        # self.get_logger().info("WAITING FOR NAVIGATION SERVER...")
        self.ac.wait_for_server()
        # self.get_logger().info("NAVIGATION SERVER AVAILABLE...")
        # self.get_logger().info("SENDING GOAL TO NAVIGATION SERVER...")
        self.start_time = time.time()

        self.goal_future = self.ac.send_goal_async(self.goal)

    def make_victim(self, pose):
        """
        This creates a new victim in the robot's world-knowledge and hopefully doesn't create repeats. Returns true if the victim was new.
        """
        victim_pose = tf2_geometry_msgs.PoseStamped()
        victim_pose.header.frame_id = "camera_rgb_optical_frame"
        victim_pose.pose = pose
        try:
            trans_pose = self.buffer.transform(victim_pose, "map")

            orient = orient = trans_pose.pose.orientation
            orientation_list = np.array(
                [orient.x, orient.y, orient.z, orient.w])
            vic_orient = tf_transformations.euler_from_quaternion(
                orientation_list)
            
            victim_pose.pose = trans_pose.pose
            trans_point_stamped = PointStamped()
            trans_point_stamped.point = trans_pose.pose.position
            trans_point_stamped.header.frame_id = "camera_rgb_optical_frame"

            if not self.duplicate_victim(trans_pose.pose.position, self.victim_messages):
                self.get_logger().info(
                    f"NEW VICTIM FOUND AT: X = {victim_pose.pose.position.x: .02f}, Y = {victim_pose.pose.position.y: .02f} THETA = {vic_orient[2]: .02f}")
                victim_info = VictimMsg()
                victim_info.id = self.victim_count
                self.victim_count += 1
                victim_info.point = trans_point_stamped
                # victim_info.description = vn.give_name() # gives a random name from victim_namer.py. Completely useless to the function of the program.
                victim_info.description = "Jonald the Journalist"

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
    
    def valid_victim(self, vic_pose: Pose):
        """
        Only accepts victims that are within a certain distance of the robot and also in valid positions.
        """
        if self.current_pose is not None:
            dist_threshold = 1.5
            vic_point = vic_pose.position
            # tim_pose = tf2_geometry_msgs.PoseStamped()
            # tim_pose.header.frame_id = "camera_rgb_optical_frame"
            # tim_pose.pose = vic_pose
            # trans_pose = self.buffer.transform(tim_pose, "map")
            # height_check = trans_pose.pose.position.z > 0.3 and trans_pose.pose.position.z < .34

            distance = np.linalg.norm(np.array([0, 0]) - np.array([vic_point.x, vic_point.y]))
            # self.get_logger().info(f"Distance to possible victim: {distance: .02f}")
            if distance < dist_threshold:
                return True
        return False

    def duplicate_victim(self, new_point : Point, existing_victims):
        """
        Checks if a victim is a duplicate, and returns true only if it *is* a duplicate
        """
        threshold = 0.47
        for victim in existing_victims:
            distance = np.linalg.norm(
                np.array([victim.point.point.x, victim.point.point.y]) -
                np.array([new_point.x, new_point.y])
            )
            if distance < threshold:
                x_avg = (victim.point.point.x + new_point.x) / 2
                y_avg = (victim.point.point.y + new_point.y) / 2
                victim.point.point.x = x_avg
                victim.point.point.y = y_avg
                return True
        return False

    def search_for_victims(self):
        """
        Checks if there are more detected victims and, if so, navs toward them to get a picture.
        """
        i = self.victim_search_id
        if i < len(self.victim_poses):
            self.going_to_victim = True
            vic_pose : PoseStamped = self.victim_poses[i]
            orient = vic_pose.pose.orientation
            orientation_list = np.array([orient.x, orient.y, orient.z, orient.w])
            vic_orient = tf_transformations.euler_from_quaternion(orientation_list)
            yaw = vic_orient[2]
            if yaw >= 0:
                yaw -= np.pi
            else:
                yaw += np.pi
            # here we get (0, 1, 0) in the victim's coordinate frame, facing the victim, in the map's coordinate frame.
            goal = self.victim_transform(vic_pose, i)
            next_goal = create_nav_goal(
                goal[0], goal[1], yaw - (np.pi / 2))  # X, Y, Theta Z
            self.update_goal(next_goal)

    def victim_transform(self, pose : PoseStamped, id):
        """
        Transform (0, .75, 0) in victim's coordinate frame into a usable pose in the map's coordinate frame.
        """
        tran = transformer.Transformer("map")
        posit = pose.pose.position
        orient = pose.pose.orientation
        quat = np.array([orient.x, orient.y, orient.z, orient.w])
        euler = tf_transformations.euler_from_quaternion(quat)

        # victim to map conversion
        F_map_to_victim = transformer.trans(
            posit.x, posit.y, 0) @ transformer.rot_z(euler[2])  # yaw is euler[2]
        tran.add_transform("map", f"victim{id: d}", F_map_to_victim)

        one_ahead_of_victim = np.array([0, -0.75, 0]) # negative due to rotation to face the victim
        goal = tran.transform(f"victim{id: d}", "map", one_ahead_of_victim)
        return goal

    def shoot_photo(self):
        """
        Shoots a photograph of the victim, sets going_to_victim to false, and adds the victim to the victims_complete list
        """
        self.going_to_victim = False
        self.get_logger().info(
            f"Len victim_messages: {len(self.victim_messages): d}")
        self.get_logger().info(f"Search ID: {self.victim_search_id: d}")
        victim = self.victim_messages[self.victim_search_id]
        victim.image = self.taken_picture
        self.victims_complete.append(victim)
        self.victim_search_id += 1
        self.get_logger().info("Picture shot")

    def do_next_navigation(self):
        """
        Logic for determining which goal to go to next.
        """
        victims_remaining = len(self.victims_complete) < len(self.victim_messages)
        self.get_logger().info(
            f"NUMBER OF VICTIMS LEFT: {len(self.victim_messages) - len(self.victims_complete): d}")
        time_remaining = self.time_elapsed < self.timeout
        if victims_remaining and time_remaining:
            self.get_logger().info("GOING TO NEXT KNOWN VICTIM")
            self.search_for_victims()
        elif time_remaining:
            self.get_logger().info("RESCUEBOT CONTINUING SEARCH IN NEW POSITION")
            new_goal = self.make_new_map_goal()
            self.update_goal(new_goal)
        else:
            # hacky way to end the movement, fix later
            self.completed_navs = 1000000000000000

    def scanner_callback(self, aruco_msg: ArucoMarkers):
        """
        Scans for victims within visual sight range.
        """
        if self.wandering and self.scan_timer > 0 and not self.going_home: # Hacky way to do it so we only do this scan when wandering
            self.scan_timer = 0
            for pose in aruco_msg.poses:
                if not self.valid_victim(pose):
                    self.get_logger().info("GHOST VICTIM DETECTED, IGNORING...")
                elif self.make_victim(pose):
                    # We are gonna want to move to the victim location
                    # Take a picture then appending the victim infromation to the victim messages array
                    if not self.going_to_victim:
                        self.goal_future.result().cancel_goal_async()
                        self.going_to_victim = True
                        self.search_for_victims()
                        self.get_logger().info(f"GOING TO NEW VICTIM")

    def wander_callback(self):
        """
        This timer is keeping track of the overall status of the node's search.
        """
        if self.time_elapsed > 2: # Wait 2 seconds before we start so that the initial pose can be properly set.
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
                    else:
                        just_explored = self.goal.pose.pose.position
                        self.explored_goals.append(just_explored)
                    self.do_next_navigation()

            elif not self.wandering:
                self.get_logger().info("RESCUEBOT ROLLING OUT")
                self.start_wandering()
        self.time_elapsed += 1
        # self.get_logger().info(f"TIME ELAPSED: {self.time_elapsed: d}")

    def navigation_callback(self):
        """
        This timer will periodically check in on the progress of navigation, ten times per second.
        """
        time_remaining = self.time_elapsed < self.timeout
        if not self.going_home and not time_remaining:
            self.goal_future.result().cancel_goal_async()
            self.get_logger().info("TIME IS UP! GO HOME!")
            self.completed_navs = 10000000000000

        if not self.goal_future.done():
            i = 0
            # self.get_logger().info("NAVIGATION GOAL NOT YET ACCEPTED")

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
                    # self.node_future.set_result(False)
                    self.navigation_complete = True

            elif self.going_home:
                if self.goal_future.result().status == GoalStatus.STATUS_SUCCEEDED:
                    self.node_future.set_result(True)

            # elif time.time() - self.start_time > self.timeout:
            #     self.get_logger().info("TAKING TOO LONG. CANCELLING GOAL!")
            #     self.cancel_future = self.goal_future.result().cancel_goal_async()
        # self.scan_timer += 1

    def pose_callback(self, amcl_pose: PoseWithCovarianceStamped):
        self.current_pose = amcl_pose
        self.scan_timer += 1
    
    def initial_pose_callback(self, initial_pose: PoseWithCovarianceStamped):
        self.initial_pose = initial_pose
        self.current_pose = initial_pose
        self.explored_goals.append(initial_pose.pose.pose.position)
    
    def map_callback(self, map_msg : OccupancyGrid):
        """
        Processes the Map message.
        """
        self.get_logger().info("HEY WE GOT A MAP MESSAGE HURRAY")
        if self.map is None:  # No need to do this every time map is published.

            self.map = map_utils.Map(map_msg)

            # Use numpy to calculate some statistics about the map:
            total_cells = self.map.width * self.map.height
            pct_occupied = np.count_nonzero(self.map.grid == 100) / total_cells * 100
            pct_unknown = np.count_nonzero(self.map.grid == -1) / total_cells * 100
            pct_free = np.count_nonzero(self.map.grid == 0) / total_cells * 100
            map_str = "Map Statistics: occupied: {:.1f}% free: {:.1f}% unknown: {:.1f}%"
            self.get_logger().info(map_str.format(pct_occupied, pct_free, pct_unknown))
    
    # def costmap_callback(self, costmap_msg : OccupancyGrid):
    #     self.get_logger().info("HEY LOSERS THE COSTMAP HAS BEEN UPDATED")
    #     if self.costmap is None:  # No need to do this every time map is published.

    #         self.costmap = map_utils.Map(costmap_msg)

    #         # Use numpy to calculate some statistics about the map:
    #         total_cells = self.costmap.width * self.costmap.height
    #         pct_occupied = np.count_nonzero(self.costmap.grid == 100) / total_cells * 100
    #         pct_unknown = np.count_nonzero(self.costmap.grid == -1) / total_cells * 100
    #         pct_free = np.count_nonzero(self.costmap.grid == 0) / total_cells * 100
    #         map_str = "Map Statistics: occupied: {:.1f}% free: {:.1f}% unknown: {:.1f}%"
    #         self.get_logger().info(map_str.format(pct_occupied, pct_free, pct_unknown))
    #     else:
    #         self.costmap = map_utils.Map(costmap_msg)
    #         self.get_logger().info("HEY LOSERS THE COSTMAP HAS BEEN UPDATED")
    
    
    def make_new_map_goal(self):
        """
        Generates a new goal within the map by picking a random cell in the map's grid and comparing the X and Y of that position to a list of previously-reached positions
        """
        map = self.map
        random_iterations = 0
        while True:
            row = random.randint(-map.width, map.width) # random row within the map's cells
            col = random.randint(-map.height, map.height) # random column within the map's cells

            orient = self.current_pose.pose.pose.orientation
            orientation_array = np.array([orient.x, orient.y, orient.z, orient.w])
            euler = tf_transformations.euler_from_quaternion(orientation_array)
            rand_rot = random.uniform(-np.pi / 2, np.pi / 2)
            theta = euler[2] + rand_rot # Current orientation plus or minus anything from 0 to a quarter rotation. Totally arbitrary amount.

            cell = map.cell_position(row, col) # X and Y tuple of the center of our randomly chosen cell
            if self.good_goal(cell[0], cell[1]):
                self.get_logger().info("GOAL GENERATED")
                self.near_check_iterations = 0
                new_goal = create_nav_goal(cell[0], cell[1], theta)
                return new_goal
            
            random_iterations += 1
            # Check to lower the too_close threshold if we've done too many random_iterations without a new goal
            if random_iterations >= self.random_iterations_threshold:
                self.close_threshold = self.close_threshold / 2
                self.get_logger().info("TOO MANY BAD GOALS, REDUCING CLOSENESS THRESHOLD")
                random_iterations = 0

            # else:
            #     self.get_logger().info("Invalid attempted random goal. Generating new possible goal position...")

    def good_goal(self, x, y):
        """
        Check to make sure a goal is good to navigate to. Only return true if the position is free and it is 
        not too close to a previously-chosen random goal.
        """
        goal_string = goal_type(x, y, self.map)
        if goal_string == "free":
            prev_goals = self.explored_goals
            for prev_goal in prev_goals:
                if self.too_close(x, y, prev_goal):
                    return False
            checks_left = self.near_check_iterations < self.near_check_threshold
            if checks_left:
                # self.get_logger().info("CHECKING IF CLOSE ENOUGH...")
                near = self.near_self(x, y)
                self.near_check_iterations += 1
                if near: # if the goal we've found is close enough to ourself to be worth navigating to
                    self.near_check_iterations = 0
                    return True
                else: # if we are not out of near_checks but this particular one was not the play
                    return False
            else: # if we are out of near_checks and just want any random point that hasn't been kicked out by the too_close check
                return True
        else:
            return False
    
    def near_self(self, x, y):
        """
        Returns true only if the point we've generated is close enough to our current position.
        """
        dist_threshold = 1.0
        current_pos : Point = self.current_pose.pose.pose.position
        distance = np.linalg.norm(
                np.array([x, y]) -
                np.array([current_pos.x, current_pos.y])
            )
        # return true if the distance is near ourself, aka the distance is less than the threshold.

        # self.get_logger().info(f"Distance to position: {distance}")
        return distance < dist_threshold

    def too_close(self, x, y, prev_goal: Point):
        """
        Returns true only if we are too close to a point we have already completed a navigation to.
        """
        # dist_threshold = 2.0
        distance = np.linalg.norm(np.array([x, y]) -np.array([prev_goal.x, prev_goal.y])
            )
        # return true if the distance is too close, aka the distance is less than the threshold.
        return distance < self.close_threshold

def goal_type (x, y, map: Map):
    """
    Used to check if a point in a map is free and returns a string representing what 
    that cell's value is in the map.
    """
    free = "NOTHING, ERROR"
    val = map.get_cell(x, y)
    if val == 100:
        free = "occupied"
    elif val == 0:
        free = "free"
    else:
        free = "unknown"
    return free

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

def main():
    rclpy.init()

    # test parameter for determining how long to randomly wander, change later
    iterations = 2
    node = RescueNode(iterations)
    node_future = node.get_future()
    rclpy.spin_until_future_complete(node, node_future)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
