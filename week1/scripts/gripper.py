#! /usr/bin/env python3

import rospy
import actionlib
from control_msgs.msg import GripperCommandAction, GripperCommandGoal

ACTION_NAME = 'gripper_controller/gripper_action'
CLOSED_POS = 0.0  # The position for a fully-closed gripper (meters).
OPENED_POS = 0.10  # The position for a fully-open gripper (meters).


class Gripper(object):
    """Gripper controls the robot's gripper.
    """
    MIN_EFFORT = 35  # Min grasp force, in Newtons
    MAX_EFFORT = 100  # Max grasp force, in Newtons

    def __init__(self):
        # TODO: Create actionlib client
        self.client = actionlib.SimpleActionClient(ACTION_NAME, GripperCommandAction)

        # TODO: Wait for server
        self.client.wait_for_server()

    def open(self):
        """Opens the gripper.
        """
        # TODO: Create goal
        goal = GripperCommandGoal()
        goal.command.position = OPENED_POS

        # TODO: Send goal
        self.client.send_goal(goal)

        # TODO: Wait for result
        self.client.wait_for_result()

    def close(self, max_effort=MAX_EFFORT):
        """Closes the gripper.

        Args:
            max_effort: The maximum effort, in Newtons, to use. Note that this
                should not be less than 35N, or else the gripper may not close.
        """

        if max_effort < 35 or max_effort > 100:
            rospy.logerr('Wrong force!')

        # TODO: Create goal
        goal = GripperCommandGoal()
        goal.command.position = CLOSED_POS
        goal.command.max_effort = max_effort

        # TODO: Send goal
        self.client.send_goal(goal)

        # TODO: Wait for result
        self.client.wait_for_result()