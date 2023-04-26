#!/usr/bin/env python3

import os
import rospy
import pickle
import actionlib
from map_annotator import PoseStore
from map_annotator.msg import PoseNames, UserAction
from geometry_msgs.msg import Pose, PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, Marker, InteractiveMarkerControl, InteractiveMarkerFeedback


class PoseServer:

    def __init__(self):
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
        int_marker.scale = 1

        # Move control
        move_control = InteractiveMarkerControl()
        move_control.orientation.w = 1
        move_control.orientation.x = 0
        move_control.orientation.y = 1
        move_control.orientation.z = 0
        move_control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        int_marker.controls.append(move_control)

        # Arrow marker
        arrow_marker = Marker()
        arrow_marker.type = Marker.ARROW
        arrow_marker.pose.position.z = 0.1  # Slightly off the ground
        arrow_marker.pose.orientation.w = 1.0
        arrow_marker.scale.x = 1.0
        arrow_marker.scale.y = 0.1
        arrow_marker.scale.z = 0.1
        arrow_marker.color.r = 1.0
        arrow_marker.color.g = 0.5
        arrow_marker.color.b = 0.0
        arrow_marker.color.a = 1.0

        move_control.markers.append(arrow_marker)
        move_control.always_visible = True
        int_marker.controls.append(move_control)

        # Rotation control
        rotation_control = InteractiveMarkerControl()
        rotation_control.orientation.w = 1
        rotation_control.orientation.x = 0
        rotation_control.orientation.y = 1
        rotation_control.orientation.z = 0
        rotation_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(rotation_control)

        self.marker_server.insert(int_marker, self.handle_marker_feedback)
        self.marker_server.setCallback(int_marker.name, self.handle_marker_feedback, InteractiveMarkerFeedback.POSE_UPDATE)
        self.marker_server.applyChanges()


    def handle_marker_feedback(self, feedback):
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            pose = feedback.pose
            pose.position.x = round(pose.position.x - 0.5) + 0.5
            pose.position.y = round(pose.position.y - 0.5) + 0.5

            rospy.loginfo("{}: aligning position = {}, {}, {} to {}, {}, {}".format(
                feedback.marker_name,
                feedback.pose.position.x,
                feedback.pose.position.y,
                feedback.pose.position.z,
                pose.position.x,
                pose.position.y,
                pose.position.z
            ))

            self.marker_server.setPose(feedback.marker_name, pose)
            self.marker_server.applyChanges()

            rospy.loginfo(f"New pose for {feedback.marker_name}: {feedback.pose}")
            self.store.overwrite(feedback.marker_name, feedback.pose)


    def handle_goto(self, pose_name):
        pose = self.store.get(pose_name)
        if pose:
            move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
            move_base_client.wait_for_server()

            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose = pose

            move_base_client.send_goal(goal)
            move_base_client.wait_for_result()


    def handle_user_action(self, user_action):

        if user_action.command == "add":
            pose = Pose()
            pose.position.x = 0
            pose.position.y = 0
            pose.position.z = 0
            pose.orientation.x = 0
            pose.orientation.y = 0
            pose.orientation.z = 0
            pose.orientation.w = 1

            self.store.add(user_action.name, pose)
            self.create_interactive_marker(user_action.name, pose)
            self.publish_pose_names()

        elif user_action.command == "delete":
            self.store.delete(user_action.name)
            self.marker_server.erase(user_action.name)
            self.marker_server.applyChanges()
            self.publish_pose_names()

        elif user_action.command == "go_to":
            self.handle_goto(user_action.name)


if __name__ == "__main__":
    rospy.init_node('map_annotator_server')
    pose_server = PoseServer()
    rospy.spin()
    