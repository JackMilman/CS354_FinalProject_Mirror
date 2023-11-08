"""
This module contains the RescueNode class. It is used to represent a robot which is trying to 
randomly go to a location in space and come back to the original location

Author: Walker Todd, Jesse Yao, Jack Posada, and Jack Milman
"""
import random
import numpy as np

import rclpy
import rclpy.node
from rclpy.action.client import ActionClient
from rclpy.task import Future
import math

class RescueNode:

    def __init__(self, x, y, theta, timeout):
        super().__init__('random_nav')

        # This QOS Setting is used for topics where the messages
        # should continue to be available indefinitely once they are
        # published. Maps fall into this category.  They typically
        # don't change, so it makes sense to publish them once.
        latching_qos = QoSProfile(depth=1,
                                  durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        self.create_subscription(OccupancyGrid, 'map',
                                 self.map_callback,
                                 qos_profile=latching_qos)
        
        self.goal = create_nav_goal(x, y, theta)

        # Create the action client.
        self.ac = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # Used if we decide to cancel a goal request. None indicates
        # we haven't canceled.
        self.cancel_future = None
        
        self.map = None
        self.timeout = timeout
    
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


def main():
    while (1):
        x = random.uniform (-2.0, 2.0)
        y = random.uniform (-2.0, 2.0)
        theta = random.uniform (-np.pi, np.pi)
        timeout = float('inf')
        rclpy.init()
        node = RandomNavNode(x, y, theta, timeout)
        future = node.send_goal()
        rclpy.spin_until_future_complete(node, future)
        node.get_logger().info("Node's future: " + str(future.result()))
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
