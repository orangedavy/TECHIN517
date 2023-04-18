#! /usr/bin/env python3

import tf
import copy
import math
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class Base(object):
    """Base controls the mobile base portion of the Fetch robot.

    Sample usage:
        base = robot_api.Base()
        while CONDITION:
            base.move(0.2, 0)
        base.stop()
    """

    def __init__(self):
        self._odom = None
        self._odom_sub = rospy.Subscriber('odom', Odometry, callback=self._odom_callback)
        # TODO: Create publisher
        self._vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def _odom_callback(self, msg):
        self._odom = msg

    def get_odom(self):
        return self._odom

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
        self._vel_pub.publish(motion_move)

    def stop(self):
        """Stops the mobile base from moving.
        """
        # TODO: Publish 0 velocity
        motion_stop = Twist()
        motion_stop.linear.x = 0.0
        motion_stop.angular.z = 0.0
        self._vel_pub.publish(motion_stop)

    def go_forward(self, distance, speed=0.1):
        """Moves the robot a certain distance.

        It's recommended that the robot move slowly. If the robot moves too
        quickly, it may overshoot the target. Note also that this method does
        not know if the robot's path is perturbed (e.g., by teleop). It stops
        once the distance traveled is equal to the given distance or more.

        Args:
            distance: The distance, in meters, to move. A positive value
                means forward, negative means backward.
            speed: The speed to travel, in meters/second.
        """
        # TODO: rospy.sleep until the base has received at least one message on /odom
        while not self._odom:
            rospy.sleep(0.1)

        # TODO: record start position, use Python's copy.deepcopy
        start = copy.deepcopy(self._odom)
        rate = rospy.Rate(10)

        def euclidean_distance(pose1, pose2):
            return math.sqrt((pose1.position.x - pose2.position.x) ** 2 +
                             (pose1.position.y - pose2.position.y) ** 2)

        # TODO: CONDITION should check if the robot has traveled the desired distance
        REACHED_DISTANCE = False

        # TODO: Be sure to handle the case where the distance is negative!
        while not REACHED_DISTANCE:

            # TODO: you will probably need to do some math in this loop to check the CONDITION
            traveled_distance = euclidean_distance(start.pose.pose, self._odom.pose.pose)
            remaining_distance = abs(distance) - traveled_distance

            if remaining_distance <= 0:
                REACHED_DISTANCE = True

            speed = max(0.25, min(speed, abs(speed)))
            direction = -1 if distance < 0 else 1
            self.move(direction * speed, 0)
            rate.sleep()

    def get_yaw(self, odom):
        quaternion = (
            odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z,
            odom.pose.pose.orientation.w,
        )
        _, _, yaw = tf.transformations.euler_from_quaternion(quaternion)
        return yaw

    def turn(self, distance, speed=0.5):
        """Rotates the robot a certain angle.

        Args:
            angular_distance: The angle, in radians, to rotate. A positive
                value rotates counter-clockwise.
            speed: The angular speed to rotate, in radians/second.
        """
        # TODO: rospy.sleep until the base has received at least one message on /odom
        while not self._odom:
            rospy.sleep(0.1)

        # TODO: record start position, use Python's copy.deepcopy
        start = copy.deepcopy(self._odom)
        start_yaw = self.get_yaw(start)

        # TODO: What will you do if angular_distance is greater than 2*pi or less than -2*pi?
        if distance > 2 * math.pi:
            distance %= 2 * math.pi
        elif distance < -2 * math.pi:
            distance %= -2 * math.pi
        
        rate = rospy.Rate(5)

        # TODO: CONDITION should check if the robot has rotated the desired amount
        REACHED_DISTANCE = False

        # TODO: Be sure to handle the case where the desired amount is negative!
        while not REACHED_DISTANCE:

            # TODO: you will probably need to do some math in this loop to check the CONDITION
            current_yaw = self.get_yaw(self._odom)
            traveled_distance = current_yaw - start_yaw

            # Normalize the traveled distance to be between -pi and pi using math.atan2
            traveled_distance = math.atan2(math.sin(traveled_distance), 
                                           math.cos(traveled_distance))
            
            remaining_distance = distance - traveled_distance

            # Choose the shortest path by considering the wrapped angles
            if remaining_distance > math.pi:
                remaining_distance -= 2 * math.pi
            elif remaining_distance < -math.pi:
                remaining_distance += 2 * math.pi

            # A tolerance to consider the rotation complete
            if abs(remaining_distance) <= 0.01:
                REACHED_DISTANCE = True

            speed = max(0.15, min(speed, abs(speed)))
            direction = -1 if remaining_distance < 0 else 1
            self.move(0, direction * speed)
            rate.sleep()

        