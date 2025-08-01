#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
import math
import os

# Global state variables
recording = False
last_button_state = 0
origin_position = None
last_position = None
positions = []
filename = os.path.expanduser("~/recorded_path.txt")

def joystickCallback(data):
    global recording, origin_position, last_position, positions

    new_recording_state = bool(data.buttons[2])

    if new_recording_state != recording:
        recording = new_recording_state
        if recording:
            rospy.loginfo("Recording started")
            # Reset for new recording session
            origin_position = None
            last_position = None
            positions = [(0.0, 0.0)]  # start with origin
        else:
            rospy.loginfo("Recording stopped")
            save_to_file()


def odom_callback(msg):
    global recording, origin_position, last_position, positions

    if not recording:
        return

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    # Set the origin at the moment recording starts
    if origin_position is None:
        origin_position = (x, y)
        last_position = (0.0, 0.0)  # local origin
        return

    # Relative to origin
    rel_x = x - origin_position[0]
    rel_y = y - origin_position[1]
    current_rel = (rel_x, rel_y)

    dist = math.hypot(current_rel[0] - last_position[0], current_rel[1] - last_position[1])
    if dist >= 0.1:
        positions.append(current_rel)
        last_position = current_rel

def save_to_file():
    global positions, filename

    try:
        with open(filename, 'w') as f:
            for x, y in positions:
                f.write("{:.2f},{:.2f}\n".format(x, y))
        rospy.loginfo("Path saved to: %s" % filename)
    except Exception as e:
        rospy.logerr("Failed to write to file: %s" % str(e))

def main():
    rospy.init_node('odom_recorder', anonymous=True)

    rospy.Subscriber('/odom', Odometry, odom_callback)
    rospy.Subscriber('/joy', Joy, joystickCallback)

    rospy.loginfo("Odom recorder node started.")
    rospy.spin()

if __name__ == '__main__':
    main()
