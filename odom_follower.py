#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from std_msgs.msg import Empty
import math
import os

filename = os.path.expanduser("~/recorded_path.txt")

# Global variables (DO NOT CHANGE THESE)
positions = []
current_waypoint = 0
pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
command = Twist()
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
    #print(f"Current position: {position}, yaw: {yaw}")  # Debug print

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
    except Exception as e:
        positions = []

def distance_to_waypoint():
    """Calculate distance to current waypoint"""
    if current_waypoint < len(positions):
        target_x, target_y = positions[current_waypoint]
        return math.hypot(target_x - position[0], target_y - position[1])
    return 0.0

def turn_around():
    """Make the robot turn 180 degrees"""
    global yaw, command
    
    rate = rospy.Rate(10)
    target_yaw = yaw + math.pi  # Add 180 degrees
    # Normalize angle to [-pi, pi]
    target_yaw = (target_yaw + math.pi) % (2 * math.pi) - math.pi
    
    print("Starting turn around maneuver")
    
    while not rospy.is_shutdown() and abs(yaw - target_yaw) > 0.1:
        # Simple P controller for rotation
        error = (target_yaw - yaw + math.pi) % (2 * math.pi) - math.pi
        command.angular.z = 0.5 * error
        command.linear.x = 0.0  # Ensure no linear motion during turn
        pub.publish(command)
        rate.sleep()
    
    # Stop rotation
    command.angular.z = 0.0
    pub.publish(command)
    print("Turn around complete")

def follow_path():
    """Main control loop to follow the path"""
    global current_waypoint, command, returning_home, position, yaw
    
    if not positions:
        print("No waypoints loaded - cannot follow path")
        return
    
    rate = rospy.Rate(10)  # 10 Hz
    
    
    # First follow the path to the destination
    while not rospy.is_shutdown() and current_waypoint < len(positions):
        target_x, target_y = positions[current_waypoint]
        
        # Calculate distance and angle to waypoint
        dx = target_x - position[0]
        dy = target_y - position[1]
        distance = math.hypot(dx, dy)
        angle = math.atan2(dy, dx)
        
        # Calculate angle difference (considering circular nature of angles)
        angle_diff = (angle - yaw + math.pi) % (2 * math.pi) - math.pi
        
        #print(f"Waypoint {current_waypoint}: Dist={distance:.2f}, AngleDiff={angle_diff:.2f}")
        
        if distance > waypoint_threshold:
            # Calculate velocities (simple P-controller)
            linear_vel = min(0.2, 0.5 * distance)  # Cap at 0.2 m/s
            angular_vel = 0.8 * angle_diff  # More aggressive turning
            
            # Publish velocity command
            command.linear.x = linear_vel
            command.angular.z = angular_vel
            pub.publish(command)
        else:
            # Reached the waypoint
            current_waypoint += 1
            
            # Stop briefly before moving to next waypoint
            command.linear.x = 0.0
            command.angular.z = 0.0
            pub.publish(command)
            rospy.sleep(0.5)
        
        rate.sleep()
    
    # Reached destination - turn around for return trip
    if not returning_home:
        print("Reached destination - preparing return trip")
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
        command.linear.x = 0.0
        command.angular.z = 0.0
        pub.publish(command)
        print("Completed return trip - back at starting point")

def main():
    rospy.init_node('path_follower', anonymous=True)
    
    # Setup subscriber
    rospy.Subscriber('/odom', Odometry, odom_callback)
    
    # Read the path file
    read_file()
    
    # Wait briefly for everything to initialize
    rospy.sleep(1.0)
    
    # Start following the path
    print("Starting path following...")
    follow_path()
    
    rospy.spin()

if __name__ == '__main__':
    main()