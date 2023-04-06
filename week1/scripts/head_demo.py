#! /usr/bin/env python3

import rospy
import robot_api


def print_usage():
    # NOTE: We don't expect you to implement look_at for Kuri
    # But if you do, show us because that would be impressive ;)
    # `eyes`, naturally, is Kuri only.
    print(f'Usage:'
          f'    rosrun applications head_demo.py look_at FRAME_ID X Y Z'
          f'    rosrun applications head_demo.py pan_tilt PAN_ANG TILT_ANG'
          f'    rosrun applications head_demo.py eyes ANG'
          f'Examples:'
          f'    rosrun applications head_demo.py look_at base_link 1 0 0.3'
          f'    rosrun applications head_demo.py pan_tilt 0 0.707'
          f'    rosrun applications head_demo.py eyes .50')


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def main():

    rospy.init_node('head_demo')
    wait_for_time()

    argv = rospy.myargv()
    if len(argv) < 2:
        print_usage()
        return
    command = argv[1]

    if command == 'look_at':
        if len(argv) < 6:
            print_usage()
            return
        frame_id, x, y, z = argv[2], float(argv[3]), float(argv[4]), float(argv[5])
        look_at = robot_api.Head()
        look_at.look_at(frame_id, x, y, z)

    elif command == 'pan_tilt':
        if len(argv) < 4:
            print_usage()
            return
        pan, tilt = float(argv[2]), float(argv[3])
        pan_tilt = robot_api.Head()
        pan_tilt.pan_tilt(pan, tilt)

    else:
        print_usage()


if __name__ == '__main__':
    main()
