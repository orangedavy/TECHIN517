#! /usr/bin/env python3

import rospy
import robot_api
from joint_state_reader import JointStateReader


def print_usage():
    print('Reads states and values of joints of the robot')
    print('Usage: rosrun applications joint_reader_demo.py joint_1 (joint_2) ...')


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def main():

    rospy.init_node('joint_reader_demo')
    wait_for_time()

    reader = JointStateReader()
    rospy.sleep(0.5)

    argv = rospy.myargv()

    if len(argv) == 2:
        print_usage()
        name = argv[1]
        arm_val = reader.get_joint(name)
        print(f"{name}\t{arm_val}")
        return

    elif len(argv) == 1:
        print_usage()
        names = robot_api.ArmJoints.names()
        arm_vals = reader.get_joints(names)
    
    else:
        names = argv[1:]
        arm_vals = reader.get_joints(names)

    for k, v in zip(names, arm_vals):
        print(f"{k}\t{v}")


if __name__ == '__main__':
    main()