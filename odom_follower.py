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
cmd_vel_pub = rospy.Publisher("/mobile_base/commands/velocity", Twist, queue_size=10)
position = (0.0, 0.0)  # Current position
yaw = 0.0  # Current orientation
waypoint_threshold = 0.1  # Distance threshold to consider waypoint reached
returning_home = False  # Flag to track if we're returning home
counter = 0

def odom_callback(msg):
    global position, yaw
    position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
    
    # Extract yaw from quaternion
    q = msg.pose.pose.orientation
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

def read_file(): #working at least
    global positions
    try:
        with open(filename) as f:
            for line in f:
                line = line.strip()
                if line:
                    x, y = map(float, line.split(','))
                    positions.append((x, y))
                    print("position added: ", x,y)
    except Exception as e:
        positions = []

def distance_to_waypoint():
    if current_waypoint < len(positions):
        target_x, target_y = positions[current_waypoint]
        return math.hypot(target_x - position[0], target_y - position[1])
    return 0.0

def turn_around():
    global yaw
    
    rate = rospy.Rate(10)
    cmd_vel = Twist()
    
    target_yaw = yaw + math.pi  # Add 180 degrees
    # Normalize angle to [-pi, pi]
    target_yaw = (target_yaw + math.pi) % (2 * math.pi) - math.pi
    
    
    while not rospy.is_shutdown() and abs(yaw - target_yaw) > 0.1:
        # Simple P controller for rotation
        error = (target_yaw - yaw + math.pi) % (2 * math.pi) - math.pi
        cmd_vel.angular.z = 0.5 * error
        cmd_vel_pub.publish(cmd_vel)
        rate.sleep()
    
    # Stop rotation
    cmd_vel.angular.z = 0.0
    cmd_vel_pub.publish(cmd_vel)

def turn(speed, degrees):
    
    global moveMode
    
    command = Twist()
    rate = rospy.Rate(10)
    current_speed = 0.0
    direction = 1 if speed > 0 else -1

    command.linear.x = 0.0
    command.angular.z = 0.0
    velocityPub.publish(command)

    target_angle = degrees #target angle to turn
    lastAngle = curDeg #last angle from odom
    curAngle = 0.0 #tracks cumalitive angle turned

    while not rospy.is_shutdown():
        Change = handleWrap(lastAngle, curDeg)
        curAngle += abs(Change)
        lastAngle = curDeg

        angleRemaining = target_angle - curAngle

        if abs(angleRemaining) <= .5: 
            command.linear.x = 0.0
            command.angular.z = 0.0
            velocityPub.publish(command)
            rospy.sleep(0.5)
            moveMode = 0
            break
        
        if abs(current_speed) < abs(speed):
            current_speed += 0.04 * direction
            current_speed = min(abs(speed), abs(current_speed), 0.7) * direction

        if(abs(angleRemaining) < 20):
            current_speed = max(abs(angleRemaining/100), 0.05)* direction
        
        command.linear.x = 0.0
        command.angular.z = current_speed
      
        velocityPub.publish(command)
        print("Current Position: X: {}, Y: {}, Degrees: {}".format(curX, curY, curDeg))
        print("current speed: {}, angle remaining: {}".format(current_speed, angleRemaining))
        rate.sleep()

def follow_path():
    global returning_home, positions, position

    linear_speed = 0.15
    angle_threshold = math.radians(10)

    Kp, Ki, Kd = 1.5, 0.0, 0.1
    last_error = 0.0
    integral = 0.0

    rate = rospy.Rate(20)
    counter = 0  # Start at the first waypoint

    while not rospy.is_shutdown() and counter < len(positions):

        # Get the next 5 waypoints from the current position
        avg_count = min(3, len(positions) - counter)
        sum_x = sum_y = 0.0
        for i in range(avg_count):
            sum_x += positions[counter + i][0]
            sum_y += positions[counter + i][1]
        target_x = sum_x / avg_count
        target_y = sum_y / avg_count

        print("current position:", position)
        print("target position:", target_x, target_y)

        dx = target_x - position[0]
        dy = target_y - position[1]
        distance = math.hypot(dx, dy)
        target_angle = math.atan2(dy, dx)
        error = (target_angle - yaw + math.pi) % (2 * math.pi) - math.pi

        print("distance:", distance, "target angle:", target_angle, "yaw:", yaw)

        derivative = error - last_error
        integral += error
        last_error = error
        angular_z = Kp * error + Ki * integral + Kd * derivative

        cmd_vel = Twist()

        if distance > 0.1:  # Keep moving toward averaged point
            if abs(error) < angle_threshold:
                cmd_vel.linear.x = linear_speed
            

            cmd_vel.angular.z = max(-1.0, min(1.0, angular_z))

        else:
            counter += 1
            #cmd_vel.linear.x = 0.0
            #cmd_vel.angular.z = 0.0
            #cmd_vel_pub.publish(cmd_vel)
            #rospy.sleep(0.05)
            continue

        cmd_vel_pub.publish(cmd_vel)
        rate.sleep()

    # Stop robot at end
    cmd_vel_pub.publish(Twist())

    if not returning_home and len(positions) > 0:
        turn_around()
        positions.reverse()
        returning_home = True
        follow_path()




        

def main():
    global cmd_vel_pub, resetOdomPub
    
    rospy.init_node('path_follower', anonymous=True)
    
    # Setup publishers and subscribers
    resetOdomPub = rospy.Publisher('/mobile_base/commands/reset_odometry', Empty, queue_size=10)

    rospy.Subscriber('/odom', Odometry, odom_callback)
    

    
    # Read the path file
    print("before reading file")
    read_file()
    
    # Wait briefly for everything to initialize
    
    for _ in range(3):
        resetOdomPub.publish(Empty())
        rospy.sleep(0.5)    
        print("reset odometry")
    print("done sleeping")
    # Start following the path
    follow_path()
    
    rospy.spin()

if __name__ == '__main__':
    main()