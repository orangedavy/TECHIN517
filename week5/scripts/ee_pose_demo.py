#!/usr/bin/env python3

import tf
import rospy
from geometry_msgs.msg import PoseStamped

def get_gripper_pose(tf_listener):
    try:
        base_frame = 'base_link'
        end_effector_frame = 'gripper_link'
        time = rospy.Time(0)
        tf_listener.waitForTransform(base_frame, end_effector_frame, time, rospy.Duration(1.0))
        position, orientation = tf_listener.lookupTransform(base_frame, end_effector_frame, time)

        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = base_frame
        pose_stamped.header.stamp = time
        pose_stamped.pose.position.x = position[0]
        pose_stamped.pose.position.y = position[1]
        pose_stamped.pose.position.z = position[2]
        pose_stamped.pose.orientation.x = orientation[0]
        pose_stamped.pose.orientation.y = orientation[1]
        pose_stamped.pose.orientation.z = orientation[2]
        pose_stamped.pose.orientation.w = orientation[3]
        return pose_stamped

    except tf.Exception as e:
        rospy.logerr(e)
        return None


def main():
    rospy.init_node('ee_pose_demo')
    
    tf_listener = tf.TransformListener()
    rospy.sleep(0.1)
    
    rate = rospy.Rate(1)  # 1 Hz
    while not rospy.is_shutdown():
        gripper_pose = get_gripper_pose(tf_listener)
        if gripper_pose is not None:
            rospy.loginfo(f"Gripper pose: \n{str(gripper_pose)}")
        rate.sleep()


if __name__ == '__main__':
    main()
