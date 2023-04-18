#! /usr/bin/env python3

import math
import rospy
import robot_api
from nav_msgs.msg import Odometry
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point, Pose
from tf.transformations import quaternion_multiply, quaternion_inverse
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback, Marker

marker_movement = {
    'Move forward': (Point(x=0.45, y=0, z=0.5), Marker.CUBE, ColorRGBA(r=0.8, g=0.4, b=0.4, a=0.8)),
    'Move backward': (Point(x=-0.45, y=0, z=0.5), Marker.CUBE, ColorRGBA(r=0.4, g=0.8, b=0.4, a=0.8)),
    'Turn left': (Point(x=0, y=0.45, z=0.5), Marker.SPHERE, ColorRGBA(r=0.4, g=0.4, b=0.8, a=0.8)),
    'Turn right': (Point(x=0, y=-0.45, z=0.5), Marker.SPHERE, ColorRGBA(r=0.8, g=0.8, b=0.4, a=0.8)),
}


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def print_usage():                                                                            
    print(f'Usage: rosrun applications base_demo.py move 0.1'
          f'       rosrun applications base_demo.py rotate 30')


def handle_viz_input(feedback, move_func, distance):
    if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
        rospy.loginfo(feedback.marker_name + ' was clicked.')
        move_func(distance)
    else:
        rospy.loginfo('Cannot handle this InteractiveMarker event.')


def add_interactive_marker(server, position, name, shape, color, move_func, distance):
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "odom"
    int_marker.pose.position = position
    int_marker.pose.orientation.w = 1
    int_marker.name = name

    shape_marker = Marker()
    shape_marker.type = shape
    shape_marker.pose.orientation.w = 1
    shape_marker.scale.x = 0.2
    shape_marker.scale.y = 0.2
    shape_marker.scale.z = 0.2
    shape_marker.color = color

    text_marker = Marker()
    text_marker.type = Marker.TEXT_VIEW_FACING
    text_marker.text = name
    text_marker.pose.position.z = 0.2
    text_marker.pose.orientation.w = 1
    text_marker.scale.z = 0.1
    text_marker.color = color

    button_control = InteractiveMarkerControl()
    button_control.interaction_mode = InteractiveMarkerControl.BUTTON
    button_control.always_visible = True
    button_control.markers.append(shape_marker)
    button_control.markers.append(text_marker)
    int_marker.controls.append(button_control)

    server.insert(int_marker, 
                  lambda feedback: handle_viz_input(feedback, move_func, distance))
    server.applyChanges()


def update_marker_position(server, msg):
    robot_orientation = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                         msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)

    for name, (position, _, _) in marker_movement.items():
        local_position = (position.x, position.y, position.z, 0)
        global_position = quaternion_multiply(robot_orientation, 
                                              quaternion_multiply(local_position, 
                                                                  quaternion_inverse(robot_orientation)))
        new_position = Point(x=msg.pose.pose.position.x + global_position[0],
                             y=msg.pose.pose.position.y + global_position[1],
                             z=msg.pose.pose.position.z + global_position[2])

        server.setPose(name, Pose(position=new_position, orientation=msg.pose.pose.orientation))

    server.applyChanges()


def main():

    rospy.init_node('base_demo')
    wait_for_time()

    base = robot_api.Base()

    argv = rospy.myargv()
    if len(argv) >= 3:
        command = argv[1]
        value = float(argv[2])

        if command == 'move':
            base.go_forward(value)
        elif command == 'rotate':
            base.turn(value * math.pi / 180)
        else:
            print_usage()

    server = InteractiveMarkerServer("simple_marker")

    for name, (position, shape, color) in marker_movement.items():
        add_interactive_marker(
            server,
            position,
            name,
            shape,
            color,
            base.go_forward if 'Move' in name else base.turn,
            1.0 if name == 'Move forward' else -1.0 if name == 'Move backward' 
            else 0.5 * math.pi if name == 'Turn left' else -0.5 * math.pi,
        )

    rospy.Subscriber('odom', Odometry, lambda msg: update_marker_position(server, msg))
    rospy.spin()


if __name__ == '__main__':
    main()