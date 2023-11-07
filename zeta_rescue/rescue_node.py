"""
This module contains the RescueNode class. It is used to represent a robot which is trying to 
randomly go to a location in space and come back to the original location

Author: Walker Todd, Jesse Yao, Jack Posada, and Jack Milman
"""
import random

class RescueNode:

    def __init__(self, w, x):
        """Particle constructor.

        Args:
            w (float): weight for this particle
            x (int): state for this particle
        """
        self.weight = w
        self.x = x

    s
    "Move the robot to the next location."
    def move(self, odom1, odom2):
        # Get the current location.
        x = odom1.pose.pose.position.x
        y = odom1.pose.pose.position.y
        # Get the next location.
        x2 = odom2.pose.pose.position.x
        y2 = odom2.pose.pose.position.y
        # Calculate the distance between the two locations.
        distance = math.sqrt((x2 - x)**2 + (y2 - y)**2)
        # Calculate the time it will take to get to the next location.
        time = distance / self.speed
        # Move the robot to the next location.
        self.move_base(0.5, 0.0, 0.0, 0.0)
        # Sleep for the time it will take to get to the next location.
        rospy.sleep(time)
        # Stop the robot.
        self.move_base(0.0, 0.0, 0.0, 0.0)


def main():
    print('Hi from zeta_rescue.')


if __name__ == '__main__':
    main()
