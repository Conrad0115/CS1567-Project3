#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Empty, Float32


#global vars
curX = 0.0
curY = 0.0
curYaw = 0.0

#
velocityPub = None



class move:
    def __init__(self, X, Y):
        self.X = X
        self.Y = Y

def resetOdom():
    global resetOdomPub
    resetOdomPub = rospy.Publisher('/mobile_base/commands/reset_odometry', Empty,queue_size=10)
    
    while resetOdomPub.get_num_connections() == 0:
        pass
    resetOdomPub.publish(Empty())
    rospy.sleep(0.5)


def odomCallback(data):
    global curX,curY,curYaw
    q = [data.pose.pose.orientation.x,
    data.pose.pose.orientation.y,
    data.pose.pose.orientation.z,
    data.pose.pose.orientation.w]
    roll, pitch, yaw = euler_from_quaternion(q)

    curYaw = yaw 
    curX = data.pose.pose.position.x
    curY = data.pose.pose.position.y

def executeMoves(moves):
    for move in moves:
        print "Moving to: (", move.X, ",", move.Y, ")"
        dx = move.X - curX
        dy = move.Y - curY
        angle = math.atan2(dy, dx)

        rotate_to_angle(angle)
        move_to_point(move.X, move.Y)
        rospy.sleep(1.0)

def normalizeAngle(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi

def getCoordinate(x,y, theta1, theta2): 
    m1 = math.tan(theta1)
    m2 = math.tan(theta2)

    x_target = -(x*m2 + y)/(m2-m1)
    y_target = x_target*m1


    print("X target: ", x_target)
    print("Y target: ", y_target)

    return [x_target, y_target]


def getTarget(ball_x, ball_y, goal_x, goal_y, dist):
    dx =  ball_x-goal_x 
    dy =  ball_y-goal_y
    length = math.sqrt((dx)**2+(dy)**2)

    target_y = ball_y + (dist*dy)/length
    target_x = ball_x + (dist*dx)/length
    return[target_x, target_y]


def rotate_to_angle(target_angle):
    global curYaw
    command = Twist()
    rate = rospy.Rate(40)

    curSpeed = 0

    while not rospy.is_shutdown():
        angleRemaining = normalizeAngle(target_angle - curYaw)
        print "angle remaining: ", (angleRemaining *(180/math.pi))
        direction = 1 if angleRemaining > 0 else -1
        if abs(angleRemaining) < 0.02:
            break
        
        if abs(angleRemaining) < 0.7:
            curSpeed = max(abs(angleRemaining), 0.2)*direction

        elif abs(curSpeed) <= 0.7:
            curSpeed += .04*direction
            curSpeed = min(0.7, abs(curSpeed)) * direction
        
            

        command.angular.z = curSpeed
        command.linear.x = 0.0
        velocityPub.publish(command)
        rate.sleep()


    command.linear.x = 0.0
    command.angular.z = 0.0
    velocityPub.publish(command)
    rospy.sleep(0.5)



def move_to_point(target_x, target_y):
    global curX, curY, curYaw
    rate = rospy.Rate(30)
    command = Twist()
    speed = 0

    while not rospy.is_shutdown():
        print "x: ", curX, ", y: ", curY
        dx = target_x - curX
        dy = target_y - curY
        distance = math.sqrt(dx**2 + dy**2)

        if distance < 0.1:
            break

        target_angle = math.atan2(dy, dx)
        angle_error = normalizeAngle(target_angle - curYaw)

        if(distance > 0.5):
            speed = min(speed + 0.02, 0.5)
            command.linear.x = speed
        
        else:
            speed = min(speed, distance)
            speed = max(speed, 0.1)
            command.linear.x = speed

        command.angular.z = 1.2 * angle_error
        velocityPub.publish(command)
        rate.sleep()


    command.linear.x = 0
    command.angular.z = 0
    velocityPub.publish(command)

def avoidObject(destination, object_location, object_radius):
    # check if cylinder blocks path (intersects with cylinder radius + robot radius)
    # if it doesnt
        # return null
    # cos^-1(u.v/||u||*||v||)
    destination_dist = math.sqrt(destination.X**2+destination.Y**2)
    object_dist = math.sqrt(object_location.X**2+object_location.Y**2)
    theta = math.arccos((destination.X*object_location.X + destination.Y*object_location.Y) / (destination_dist * object_dist))
    object_from_route = object_dist * math.sin(theta) - object_radius
    ROBOT_RADIUS = 0.17
    needed_distance = ROBOT_RADIUS + object_radius + 0.1
    if abs(object_from_route) >= needed_distance:
        return None
    correction_distance = needed_distance - object_from_route

    
    theta3 = math.arctan(destination.Y/destination.X)
    theta4 = -(1/theta3)

    intersection = getCoordinate(object_location.X, object_location.Y, theta3, theta4)

    correction_point = getTarget(intersection[0], intersection[1], object_location.X, object_location.Y, correction_distance)

    return move(correction_point[0], correction_point[1])


    

def coordinateDriver():
    global velocityPub, resetOdomPub, curX, curY, curYaw

    rospy.init_node("controller", anonymous=True)
    velocityPub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
   
    rospy.Subscriber('/odom', Odometry, odomCallback)

    #collecting the coordinates
    moves = []
    print("Enter move in format: 'X, Y'. Ex. '2.3, 4.1' or 'q' to quit")
    destination = raw_input("Enter Move: ").strip()
    print("Enter object position in format: 'X, Y'")
    object_location = raw_input("Enter Move: ").strip()
    print("Enter object radius")
    object_radius = raw_input("Enter Move: ").strip()

    parts = [part for part in destination.split(",")]
    dest_x = float(parts[0].strip()) 
    dest_y = float(parts[1].strip())
    destMove = move(dest_x,dest_y)

    parts = [part for part in object_location.split(",")]
    obj_x = float(parts[0].strip()) 
    obj_y = float(parts[1].strip())
    objectMove = move(obj_x, obj_y)

    reroute_point = avoidObject(destMove, objectMove, object_radius)
    if reroute_point is not None:
        moves.append(reroute_point)
    moves.append(destMove)

    #Do the moves
    if moves: 
        resetOdom()
        print "starting position is: ", curX, ",", curY
        print "starting yaw: ", curYaw
        rospy.sleep(2)#time to set up the robot
        executeMoves(moves)   
    else:
        print("no moves to execute")   
    rospy.spin() 


if __name__ == '__main__':
    try:
        coordinateDriver()
    except rospy.ROSInterruptException:
        pass