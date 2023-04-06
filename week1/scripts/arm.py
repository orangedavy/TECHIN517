#!/usr/bin/env python3

import rospy
import actionlib
from .arm_joints import ArmJoints
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ACTION_NAME = 'arm_controller/follow_joint_trajectory'

class Arm(object):
    """Arm controls the robot's arm.

    Joint space control:
        joints = ArmJoints()
        # Fill out joint states
        arm = robot_api.Arm()
        arm.move_to_joints(joints)
    """

    def __init__(self):
        # TODO: Create actionlib client
        self.client = actionlib.SimpleActionClient(ACTION_NAME, FollowJointTrajectoryAction)

        # TODO: Wait for server
        self.client.wait_for_server()

    def move_to_joints(self, arm_joints):
        """Moves the robot's arm to the given joints.

        Args:
            arm_joints: An ArmJoints object that specifies the joint values for
                the arm.
        """
        joint_names = arm_joints.names()
        joint_values = arm_joints.values()

        # TODO: Create a trajectory point
        traj_point = JointTrajectoryPoint()

        # TODO: Set position of trajectory point
        traj_point.positions = joint_values

        # TODO: Set time of trajectory point
        traj_point.time_from_start = rospy.Duration(5)

        # TODO: Create goal
        goal = FollowJointTrajectoryGoal()

        # TODO: Add joint name to list
        goal.trajectory.joint_names = joint_names

        # TODO: Add the trajectory point created above to trajectory
        goal.trajectory.points.append(traj_point)

        # TODO: Send goal
        self.client.send_goal(goal)

        # TODO: Wait for result
        self.client.wait_for_result()
