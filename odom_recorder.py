#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
from std_msgs.msg import Empty
import math
import os

# Global state variables
recording = False
last_toggle_state = 0
last_position = None
positions = []
filename = os.path.expanduser("~/recorded_path.txt")
resetOdomPub = None




def resetOdom():
    resetOdomPub.publish(Empty())
    rospy.sleep(0.5)  
    

def recorder_callback(msg):
    global recording, last_toggle_state, last_position, positions

    toggle_recorder = msg.data

    if toggle_recorder == 1 and last_toggle_state == 0:
        if not recording: #start recording
            print("Recording STARTED.")
            recording = True
            positions = [(0.0, 0.0)]
            last_position = (0.0, 0.0)
        else: # end recording
            print("Recording STOPPED. Writing to file...")
            recording = False
            save_to_file()
            rospy.signal_shutdown("Recording completed and saved.")
    
    last_toggle_state = toggle_recorder

def odom_callback(msg):
    global recording, last_position, positions

    if not recording:
        return

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    current = (x, y)

    if last_position is None:
        positions.append(current)
        last_position = current
    else:
        dist = math.hypot(x - last_position[0], y - last_position[1])
        if dist >= 0.1:
            positions.append(current)
            last_position = current

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
    global resetOdomPub
    rospy.init_node('odom_recorder', anonymous=True)
    resetOdomPub = rospy.Publisher('/mobile_base/commands/reset_odometry', Empty, queue_size=10)
    rospy.Subscriber('/odom', Odometry, odom_callback)
    resetOdom()

    rospy.Subscriber('/recorder_toggle', Int32, recorder_callback)

    rospy.loginfo("odom_recorder node started. Waiting for toggle messages...")
    rospy.spin()

if __name__ == '__main__':
    main()
