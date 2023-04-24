#!/usr/bin/env python3

import os
import rospy
import pickle
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, Marker, InteractiveMarkerControl, InteractiveMarkerFeedback
from map_annotator import PoseStore
from map_annotator.msg import PoseNames, UserAction
from geometry_msgs.msg import Pose, PoseStamped


class PoseServer:

    def __init__(self):
        rospy.init_node('pose_server')
        self.store = PoseStore('saved_poses.pickle')

        self.marker_server = InteractiveMarkerServer("map_annotator/map_poses")

        self.pose_names_publisher = rospy.Publisher('map_annotator/pose_names', PoseNames, queue_size=10, latch=True)
        self.user_action_subscriber = rospy.Subscriber('map_annotator/user_actions', UserAction, self.handle_user_action)

        self.publish_pose_names()
        self.create_interactive_markers()


    def publish_pose_names(self):
        pose_names = PoseNames()
        pose_names.names = list(self.store.list())
        self.pose_names_publisher.publish(pose_names)


    def create_interactive_markers(self):
        for pose_name in self.store.list():
            pose = self.store.get(pose_name)
            self.create_interactive_marker(pose_name, pose)


    def create_interactive_marker(self, name, pose):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "map"
        int_marker.name = name
        int_marker.description = name
        int_marker.pose = pose

        arrow_marker = Marker()
        arrow_marker.type = Marker.ARROW
        arrow_marker.pose.orientation.w = 1.0
        arrow_marker.scale.x = 0.5
        arrow_marker.scale.y = 0.05
        arrow_marker.scale.z = 0.05
        arrow_marker.color.r = 0.0
        arrow_marker.color.g = 1.0
        arrow_marker.color.b = 0.0
        arrow_marker.color.a = 1.0

        text_marker = Marker()
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.text = name
        text_marker.pose.position.z = 0.5
        text_marker.pose.orientation.w = 1.0
        text_marker.scale.z = 0.2
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.color.a = 1.0

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.y = 1
        control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        control.always_visible = True
        control.markers.append(arrow_marker)
        control.markers.append(text_marker)
        int_marker.controls.append(control)

        rotation_control = InteractiveMarkerControl()
        rotation_control.orientation.w = 1
        rotation_control.orientation.z = 1
        rotation_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(rotation_control)

        self.marker_server.insert(int_marker, self.handle_marker_feedback)
        self.marker_server.applyChanges()


    def handle_marker_feedback(self, feedback):
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            self.store.overwrite(feedback.marker_name, feedback.pose)


    def handle_user_action(self, user_action):
        if user_action.command == "add":
            self.store.add(user_action.name, user_action.pose)
            self.create_interactive_marker(user_action.name, user_action.pose)
            self.publish_pose_names()

        elif user_action.command == "delete":
            self.store.delete(user_action.name)
            self.marker_server.erase(user_action.name)
            self.marker_server.applyChanges()
            self.publish_pose_names()


if __name__ == "__main__":
    pose_server = PoseServer()
    rospy.spin()