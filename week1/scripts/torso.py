#!/usr/bin/env python3

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ACTION_NAME = 'torso_controller/follow_joint_trajectory'
TIME_FROM_START = 5  # How many seconds it should take to set the torso height.


class Torso(object):
    """Torso controls the robot's torso height.
    """
    MIN_HEIGHT = 0.0
    MAX_HEIGHT = 0.4

    def __init__(self):
        # TODO: Create actionlib client
        self.client = actionlib.SimpleActionClient(ACTION_NAME, FollowJointTrajectoryAction)

        # TODO: Wait for server
        self.client.wait_for_server()

        # TODO: Define torso joint name
        self.joint_name = 'torso_lift_joint'

    def set_height(self, height):
        """Sets the torso height.

        This will always take ~5 seconds to execute.

        Args:
            height: The height, in meters, to set the torso to. Values range
                from Torso.MIN_HEIGHT (0.0) to Torso.MAX_HEIGHT(0.4).
        """
        # TODO: Check that the height is between MIN_HEIGHT and MAX_HEIGHT.
        if height < self.MIN_HEIGHT or height > self.MAX_HEIGHT:
            rospy.logerr('Wrong height!')

        # TODO: Create a trajectory point
        traj_point = JointTrajectoryPoint()

        # TODO: Set position of trajectory point
        traj_point.positions = [height]

        # TODO: Set time of trajectory point
        traj_point.time_from_start = rospy.Duration(TIME_FROM_START)

        # TODO: Create goal
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = JointTrajectory()

        # TODO: Add joint name to list
        goal.trajectory.joint_names = [self.joint_name]

        # TODO: Add the trajectory point created above to trajectory
        goal.trajectory.points.append(traj_point)

        # TODO: Send goal
        self.client.send_goal(goal)

        # TODO: Wait for result
        self.client.wait_for_result()
