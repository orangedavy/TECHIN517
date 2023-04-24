#!/usr/bin/env python3

import rospy
import actionlib
from map_annotator import PoseStore
from geometry_msgs.msg import PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


def print_usage():
    print(f"Welcome to the map annotator!\n"
          f"Commands:\n"
          f"  list:           List saved poses.\n"
          f"  save <name>:    Save the robot's current pose as <name>.\n"
          f"                  Overwrites if <name> already exists.\n"
          f"  delete <name>:  Delete the pose given by <name>.\n"
          f"  goto <name>:    Sends the robot to the pose given by <name>.\n"
          f"  help:           Show this list of commands.\n"
          )


def handle_save(store, pose_name):
    pose = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped)
    store.add(pose_name, pose)
    print(f"Pose '{pose_name}' saved.")


def handle_goto(store, pose_name):
    pose = store.get(pose_name)
    if pose:
        print(f"Going to pose '{pose_name}'.")

        # Create an action client to interact with the move_base action server
        move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        move_base_client.wait_for_server()

        # Create a MoveBaseGoal with the desired pose
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = pose.pose.pose

        # Send the goal to the action server and wait for the result
        move_base_client.send_goal(goal)
        move_base_client.wait_for_result()

        # Check if the goal was reached successfully
        if move_base_client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            print(f"Reached pose '{pose_name}'.")
        else:
            print(f"Failed to reach pose '{pose_name}'.")

    else:
        print(f"No such pose '{pose_name}'.")


def handle_delete(store, pose_name):
    if store.get(pose_name):
        store.delete(pose_name)
        print(f"Pose '{pose_name}' deleted.")
    else:
        print(f"No such pose '{pose_name}'.")


def handle_list(store):
    pose_names = store.list()
    print(f"Poses: {', '.join(pose_names)}")


def main():

    rospy.init_node('pose_commander')

    argv = rospy.myargv()
    if len(argv) >= 2:

        store = PoseStore('saved_poses.pickle')
        command = argv[1]
        pose_name = argv[2] if len(argv) > 2 else None

        command_handlers = {
            'save': handle_save,
            'list': handle_list,
            'delete': handle_delete,
            'goto': handle_goto,
            'help': print_usage
        }

        if command in command_handlers:
            if command == 'list':
                command_handlers[command](store)
            elif command == 'help':
                print_usage()
            else:
                command_handlers[command](store, pose_name)
        else:
            print_usage()


if __name__ == '__main__':
    main()