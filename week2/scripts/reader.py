#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState


class JointStateReader(object):
    """Listens to /joint_states and provides the latest joint angles.

    Usage:                                                                                             
        joint_reader = JointStateReader()
        rospy.sleep(0.1)
        joint_reader.get_joint('shoulder_pan_joint')
        joint_reader.get_joints(['shoulder_pan_joint', 'shoulder_lift_joint'])
    """
    def __init__(self):
        # Initiate a data structure to store joint states
        self.joint_states = {}

        # Subscribe to /joint_states
        self.sub = rospy.Subscriber('/joint_states', JointState, self.joint_states_callback)

    def joint_states_callback(self, msg):
        for i, name in enumerate(msg.name):
            # Store joint values to dict
            self.joint_states[name] = msg.position[i]

    def get_joint(self, name):
        """Gets the latest joint value.

        Args:
            name: string, the name of the joint whose value we want to read.

        Returns: the joint value, or None if we do not have a value yet.
        """
        return self.joint_states[name] or None

    def get_joints(self, names):
        """Gets the latest values for a list of joint names.

        Args:
            name: list of strings, the names of the joints whose values we want 
                to read.

        Returns: A list of the joint values. Values may be None if we do not    
            have a value for that joint yet.
        """
        values = []

        for name in names:
            values.append(self.joint_states[name] or None)
        
        return values
