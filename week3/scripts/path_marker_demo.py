#!/usr/bin/env python3

import math
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
from nav_msgs.msg import Odometry


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


class NavPath(object):
    def __init__(self):
        self.DIST_THRESHOLD = 0.1
        self.TIME_THRESHOLD = 1.0
        self._path = []
        self._last_pose = None
        self._last_time = None
        self._pub = rospy.Publisher('visualization_marker', Marker, queue_size=5)
        self._last_pub_time = rospy.Time.now()

    def show_path_in_rviz(self):
        marker = Marker(
            type=Marker.SPHERE_LIST,
            id=0,
            lifetime=rospy.Duration(3),
            pose=Pose(Point(0.0, 0.0, 0.0), Quaternion(0, 0, 0, 1)),
            scale=Vector3(0.1, 0.1, 0.1),
            header=Header(frame_id='odom'),
            color=ColorRGBA(1.0, 0.4, 0.0, 0.8),
            points=self._path,
        )

        self._pub.publish(marker)

    def callback(self, msg):
        # rospy.loginfo(msg)

        if self._last_pose and self._last_time:
            # calculate position difference
            delta_x = msg.pose.pose.position.x - self._last_pose.pose.position.x
            delta_y = msg.pose.pose.position.y - self._last_pose.pose.position.y
            delta_z = msg.pose.pose.position.z - self._last_pose.pose.position.z
            distance = math.sqrt(delta_x ** 2 + delta_y ** 2 + delta_z ** 2)
            # calculate time difference
            delta_time = (msg.header.stamp - self._last_time).to_sec()

            if distance > self.DIST_THRESHOLD and delta_time > self.TIME_THRESHOLD:
                # append qualified position in path to mark
                self._path.append(msg.pose.pose.position)
                self.show_path_in_rviz()
                # update pose and time
                self._last_pose = msg.pose
                self._last_time = msg.header.stamp
        
        else:
            # initiate pose and time
            self._last_pose = msg.pose
            self._last_time = msg.header.stamp


def main():

    rospy.init_node('nav_path_marker')
    wait_for_time()

    nav_path = NavPath()
    rospy.sleep(0.5)
    rospy.Subscriber('odom', Odometry, nav_path.callback)

    rospy.spin()


if __name__ == '__main__':
  main()