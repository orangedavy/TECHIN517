#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def show_text_in_rviz(marker_publisher, text):
    marker = Marker(
                type=Marker.TEXT_VIEW_FACING,
                id=0,
                lifetime=rospy.Duration(3),
                pose=Pose(Point(0.0, 0.0, 1.25), Quaternion(0, 0, 0, 1)),
                scale=Vector3(0.1, 0.1, 0.1),
                header=Header(frame_id='base_link'),
                color=ColorRGBA(1.0, 0.4, 0.0, 0.8),
                text=text)
    marker_publisher.publish(marker)


def main():

    rospy.init_node('my_node')
    wait_for_time()

    marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=5)
    rospy.sleep(0.5)
    show_text_in_rviz(marker_publisher, 'Hello world!\nBye humans!')


if __name__ == '__main__':
  main()