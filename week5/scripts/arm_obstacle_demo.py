#!/usr/bin/env python3

import rospy
import robot_api
from moveit_python import PlanningSceneInterface
from moveit_msgs.msg import OrientationConstraint
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped


def setup_planning_scene(planning_scene):
    add_table(planning_scene)
    add_divider(planning_scene)


def add_table(planning_scene):
    planning_scene.removeCollisionObject('table')
    table_size_x, table_size_y, table_size_z = 0.5, 1, 0.03
    table_x, table_y, table_z = 0.8, 0, 0.6
    planning_scene.addBox('table', table_size_x, table_size_y, table_size_z, table_x, table_y, table_z)


def add_divider(planning_scene):
    planning_scene.removeCollisionObject('divider')
    size_x, size_y, size_z = 0.3, 0.01, 0.4
    x, y, z = 0.8 - (0.5 / 2) + (size_x / 2), 0, 0.6 + (0.03 / 2) + (size_z / 2)
    planning_scene.addBox('divider', size_x, size_y, size_z, x, y, z)


def create_pose(x, y, z, w):
    pose = PoseStamped()
    pose.header.frame_id = 'base_link'
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    pose.pose.orientation.w = w
    return pose


def create_orientation_constraint():
    oc = OrientationConstraint()
    oc.header.frame_id = 'base_link'
    oc.link_name = 'wrist_roll_link'
    oc.orientation.w = 1
    oc.absolute_x_axis_tolerance = 0.1
    oc.absolute_y_axis_tolerance = 0.1
    oc.absolute_z_axis_tolerance = 3.14
    oc.weight = 1.0
    return oc


def handle_result(result, pose_name):
    if result is not None:
        rospy.logerr(f'{pose_name} failed: {result}.')
    else:
        rospy.loginfo(f'{pose_name} succeeded.')


def attach_tray(planning_scene):
    frame_attached_to = 'gripper_link'
    frames_okay_to_collide_with = [
        'gripper_link', 'l_gripper_finger_link', 'r_gripper_finger_link'
    ]
    planning_scene.attachBox('tray', 0.3, 0.07, 0.01, 0.05, 0, 0,
                             frame_attached_to, frames_okay_to_collide_with)
    planning_scene.setColor('tray', 1, 0, 1)
    planning_scene.sendColors()


def cleanup_planning_scene(planning_scene):
    planning_scene.removeCollisionObject('table')
    planning_scene.removeCollisionObject('divider')
    planning_scene.removeAttachedObject('tray')


def main():
    rospy.init_node('arm_obstacle_demo')

    planning_scene = PlanningSceneInterface(frame='base_link')
    setup_planning_scene(planning_scene)

    pose1 = create_pose(0.5, -0.3, 0.75, 1)
    pose2 = create_pose(0.5, 0.3, 0.75, 1)
    oc = create_orientation_constraint()

    arm = robot_api.Arm()
    rospy.on_shutdown(lambda: arm.cancel_all_goals())

    kwargs = {
        'allowed_planning_time': 15,
        'execution_timeout': 10,
        'num_planning_attempts': 5,
        'replan': False
    }
    kwargs_mod = dict(kwargs)
    kwargs_mod['orientation_constraint'] = oc

    planning_scene.removeAttachedObject('tray')

    result = arm.move_to_pose(pose1, **kwargs)
    handle_result(result, 'Pose 1')

    attach_tray(planning_scene)

    rospy.sleep(1)

    result = arm.move_to_pose(pose2, **kwargs_mod)
    handle_result(result, 'Pose 2')

    cleanup_planning_scene(planning_scene)


if __name__ == '__main__':
    main()