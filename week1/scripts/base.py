#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

class Base(object):
    """Base controls the mobile base portion of the Fetch robot.

    Sample usage:
        base = robot_api.Base()
        while CONDITION:
            base.move(0.2, 0)
        base.stop()
    """

    def __init__(self):
        # TODO: Create publisher
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def move(self, linear_speed, angular_speed):
        """Moves the base instantaneously at given linear and angular speeds.

        "Instantaneously" means that this method must be called continuously in
        a loop for the robot to move.

        Args:
            linear_speed: The forward/backward speed, in meters/second. A
                positive value means the robot should move forward.
            angular_speed: The rotation speed, in radians/second. A positive
                value means the robot should rotate clockwise.
        """
        # TODO: Create Twist msg
        motion_move = Twist()

        # TODO: Fill out msg
        motion_move.linear.x = linear_speed
        motion_move.angular.z = angular_speed

        # TODO: Publish msg
        self.pub.publish(motion_move)

    def stop(self):
        """Stops the mobile base from moving.
        """
        # TODO: Publish 0 velocity
        motion_stop = Twist()
        motion_stop.linear.x = 0.0
        motion_stop.angular.z = 0.0
        self.pub.publish(motion_stop)
