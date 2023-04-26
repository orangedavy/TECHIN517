#!/usr/bin/env python3

import rospy
from interactive_markers.interactive_marker_server import InteractiveMarkerServer

def main():
    rospy.init_node('interactive_marker_test')
    marker_server = InteractiveMarkerServer("test_marker")
    rospy.spin()

if __name__ == "__main__":
    main()