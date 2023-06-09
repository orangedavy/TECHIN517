#! /usr/bin/env python3

import rospy
import robot_api


def print_usage():
    print('Usage: rosrun applications gripper_demo.py open')
    print('       rosrun applications gripper_demo.py close 40')


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def main():

    rospy.init_node('gripper_demo')
    wait_for_time()

    argv = rospy.myargv()
    if len(argv) < 2:
        print_usage()
        return
    command = argv[1]

    gripper = robot_api.Gripper()
    effort = gripper.MAX_EFFORT

    if command == 'close' and len(argv) > 2:
        effort = float(argv[2])

    if command == 'open':
        gripper.open()

    elif command == 'close':
        gripper.close(effort)

    else:
        print_usage()


if __name__ == '__main__':
    main()
