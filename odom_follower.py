#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from std_msgs.msg import Empty
import math
import os

filename = os.path.expanduser("~/recorded_path.txt")

# Global variables
positions = []
current_waypoint = 0
cmd_vel_pub = None
position = (0.0, 0.0)  # Current position
yaw = 0.0  # Current orientation
waypoint_threshold = 0.1  # Distance threshold to consider waypoint reached
returning_home = False  # Flag to track if we're returning home

def odom_callback(msg):
    """Callback for current position and orientation updates"""
    global position, yaw
    position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
    
    # Extract yaw from quaternion
    q = msg.pose.pose.orientation
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

def read_file():
    """Read the recorded path from file"""
    global positions
    try:
        with open(filename) as f:
            for line in f:
                line = line.strip()
                if line:
                    x, y = map(float, line.split(','))
                    positions.append((x, y))
        rospy.loginfo(f"Loaded {len(positions)} waypoints from {filename}")
    except Exception as e:
        rospy.logerr(f"Failed to read path file: {str(e)}")
        positions = []

def distance_to_waypoint():
    """Calculate distance to current waypoint"""
    if current_waypoint < len(positions):
        target_x, target_y = positions[current_waypoint]
        return math.hypot(target_x - position[0], target_y - position[1])
    return 0.0

def turn_around():
    """Make the robot turn 180 degrees"""
    global yaw
    
    rate = rospy.Rate(10)
    cmd_vel = Twist()
    
    target_yaw = yaw + math.pi  # Add 180 degrees
    # Normalize angle to [-pi, pi]
    target_yaw = (target_yaw + math.pi) % (2 * math.pi) - math.pi
    
    rospy.loginfo("Starting turn around maneuver")
    
    while not rospy.is_shutdown() and abs(yaw - target_yaw) > 0.1:
        # Simple P controller for rotation
        error = (target_yaw - yaw + math.pi) % (2 * math.pi) - math.pi
        cmd_vel.angular.z = 0.5 * error
        cmd_vel_pub.publish(cmd_vel)
        rate.sleep()
    
    # Stop rotation
    cmd_vel.angular.z = 0.0
    cmd_vel_pub.publish(cmd_vel)
    rospy.loginfo("Turn around complete")

def follow_path():
    """Main control loop to follow the path"""
    global current_waypoint, cmd_vel_pub, returning_home
    
    if not positions:
        rospy.logwarn("No waypoints loaded")
        return
    
    rate = rospy.Rate(10)  # 10 Hz
    cmd_vel = Twist()
    
    # First follow the path to the destination
    while not rospy.is_shutdown() and current_waypoint < len(positions):
        # Get current waypoint
        target_x, target_y = positions[current_waypoint]
        
        # Calculate distance and angle to waypoint
        dx = target_x - position[0]
        dy = target_y - position[1]
        distance = math.hypot(dx, dy)
        angle = math.atan2(dy, dx)
        
        # Simple proportional control
        if distance > waypoint_threshold:
            # Calculate linear and angular velocities
            linear_vel = min(0.2, distance * 0.5)  # Cap at 0.2 m/s
            angular_vel = angle * 0.5  # Proportional control for angle
            
            # Publish velocity command
            cmd_vel.linear.x = linear_vel
            cmd_vel.angular.z = angular_vel
            cmd_vel_pub.publish(cmd_vel)
        else:
            # Reached the waypoint
            rospy.loginfo(f"Reached waypoint {current_waypoint}: ({target_x:.2f}, {target_y:.2f})")
            current_waypoint += 1
            
            # Stop briefly before moving to next waypoint
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
            cmd_vel_pub.publish(cmd_vel)
            rospy.sleep(0.5)
        
        rate.sleep()
    
    # Reached destination - turn around for return trip
    if not returning_home:
        rospy.loginfo("Reached destination - preparing return trip")
        turn_around()
        
        # Reverse the path for return trip
        positions.reverse()
        current_waypoint = 0
        returning_home = True
        
        # Wait briefly before starting return
        rospy.sleep(1.0)
        
        # Follow the path back home
        follow_path()
    else:
        # Stop when done with return trip
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        cmd_vel_pub.publish(cmd_vel)
        rospy.loginfo("Completed return trip - back at starting point")

def main():
    global cmd_vel_pub
    
    rospy.init_node('path_follower', anonymous=True)
    
    # Setup publishers and subscribers
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.Subscriber('/odom', Odometry, odom_callback)
    
    # Read the path file
    read_file()
    
    # Wait briefly for everything to initialize
    rospy.sleep(1.0)
    
    # Start following the path
    follow_path()
    
    rospy.spin()

if __name__ == '__main__':
    main()