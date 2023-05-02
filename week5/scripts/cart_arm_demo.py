#!/usr/bin/env python3

import rospy
import robot_api
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped


def main():
    rospy.init_node('cart_arm_demo')

    pose1 = Pose(Point(0.042, 0.384, 1.826), Quaternion(0.173, -0.693, -0.242, 0.657))
    pose2 = Pose(Point(0.047, 0.545, 1.822), Quaternion(-0.274, -0.701, 0.173, 0.635))

    ps1 = PoseStamped()
    ps1.header.frame_id = 'base_link'
    ps1.pose = pose1

    ps2 = PoseStamped()
    ps2.header.frame_id = 'base_link'
    ps2.pose = pose2

    gripper_poses = [ps1, ps2]

    arm = robot_api.Arm()

    def shutdown():
        arm.cancel_all_goals()
    rospy.on_shutdown(shutdown)

    while not rospy.is_shutdown():
        for pose in gripper_poses:
            rospy.loginfo(f"Moving to pose: {pose}")

            result = arm.move_to_pose(pose)
            if result is not None:
                rospy.logerr(f"Error while moving to pose: {result}.")
            else:
                rospy.loginfo("Successfully moved to pose.")

            rospy.sleep(1)


if __name__ == '__main__':
    main()