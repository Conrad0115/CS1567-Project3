#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent, Led, Sound, CliffEvent, WheelDropEvent

# Publishers
velocityPub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
Led1Pub = rospy.Publisher('/mobile_base/commands/led1', Led, queue_size=1)
Led2Pub = rospy.Publisher('/mobile_base/commands/led2', Led, queue_size=1)
soundPub = rospy.Publisher('/mobile_base/commands/sound', Sound, queue_size=1)

# Globals
command = Twist()
target = Twist()
bumperActivated = True
emergency_brake = False
backwards = True
smootherMode = 1  # Eco mode
leds = True
onlyBackwards = False
received_command = False

# bumper checks
bumperLeft = 0
bumperRight = 0
bumperFront = 0

# cliff checks
cliffLeft = 0
cliffRight = 0
cliffFront = 0

# wheel drop checks
wheelDropLeft = 0
wheelDropRight = 0

def commandCallback(data):
    global bumperActivated, backwards, leds, emergency_brake, smootherMode, received_command
    bumperActivated = bool(data.data[0])
    backwards = bool(data.data[1])
    leds = bool(data.data[2])
    emergency_brake = bool(data.data[3])
    smootherMode = data.data[4]
    received_command = True
    
    eb_text = "Engaged" if emergency_brake else "Disengaged"
    mode_text = "Eco"
    if (int(smootherMode) == 0):
        mode_text = "Off"
    elif (int(smootherMode) == 1):
        mode_text = "Eco"
    else: #mode = 2
        mode_text = "Sport"
    
    print("Bumper: {}".format(bumperActivated))
    print("Backward Only: {}".format(backwards))
    print("LED and Sound: {}".format(leds))
    print("Emergency Brake: {}".format(eb_text))
    print("Smoothing Mode: {}".format(mode_text))
    print("----------------------------------------")
    

def twistCallback(data):
    global target, received_command
    target = data
    received_command = True

def bumperCallback(data):
    global bumperLeft, bumperRight, bumperFront, onlyBackwards
    if not bumperActivated:
        return

    if data.bumper == 0:
        
        if data.state == 1:
            bumperLeft = 1 
        else:
            bumperLeft = 0
        
    elif data.bumper == 1:
        if data.state == 1:
            bumperFront = 1 
        else:
            bumperFront = 0
      
    elif data.bumper == 2:
        if data.state == 1:
            bumperRight = 1 
        else:
            bumperRight = 0
        

    if (bumperLeft + bumperRight + bumperFront) > 0:
        emergencyBrake()
        onlyBackwards = True 
    else:
        onlyBackwards = False

def cliffCallback(data):
    global cliffLeft, cliffRight, cliffFront, onlyBackwards
    
    
    if data.sensor == 0:
        if data.state == 1:
            cliffLeft = 1
        else:
            cliffLeft = 0
    elif data.sensor == 1:
        if data.state == 1:
            cliffFront = 1
        else:
            cliffFront = 0
    else:
        if data.state == 1:
            cliffRight = 1
        else:
            cliffRight = 0
    
    
    # Check if any cliff sensor is triggered
    if (cliffLeft + cliffRight + cliffFront) > 0:
        rospy.loginfo("CLIFF DETECTED - EMERGENCY BRAKE ACTIVATED")
        emergencyBrake()
        onlyBackwards = True
    else:
        onlyBackwards = False

def wheelDropCallback(data):
    global wheelDropLeft, wheelDropRight, onlyBackwards
    
    # Log the wheel drop status
    if data.wheel == 0:
        if data.state == 1:
            wheelDropLeft = 1
            rospy.loginfo("left wheel data")
        else:
            wheelDropLeft = 0
            rospy.loginfo("right wheel data")

    else:
        if data.state == 1:
            wheelDropRight = 1
        else:
            wheelDropRight = 0
        
    # Check if any wheel is dropped
    if (wheelDropLeft + wheelDropRight) > 0:
        rospy.loginfo("WHEEL DROPPED - EMERGENCY BRAKE ACTIVATED")
        emergencyBrake()
        onlyBackwards = True
    else:
        onlyBackwards = False

def emergencyBrake():
    global command
    rospy.loginfo("EBRAKE")
    command.linear.x = 0.0
    command.angular.z = 0.0
    velocityPub.publish(command)

def flashBackwards():
    if not leds:
        return
    led = Led()
    sound = Sound()
    pub_sound = rospy.Publisher('/mobile_base/commands/sound', Sound, queue_size=10)
    for x in range(3,-1,-1):
        led.value = x
        Led1Pub.publish(led)
        rospy.sleep(0.1)
        Led2Pub.publish(led)
    for x in range (0,7):
        sound.value = x
        pub_sound.publish(sound)

def changeLEDs(data):
    led = Led()
    if data == 0 or not leds: # off
        led.value = 0 # black
    elif data == 1: # eco
        led.value = 1 # green
    elif data == 2: # sport
        led.value = 3 # red
    Led1Pub.publish(led)
    Led2Pub.publish(led)

def main():
    global command, smootherMode, received_command, target, onlyBackwards, emergency_brake

    rospy.init_node("smoother", anonymous=True)

    rospy.Subscriber('/robot_twist', Twist, twistCallback)
    rospy.Subscriber('/robot_commands', Int32MultiArray, commandCallback)
    rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, bumperCallback)
    rospy.Subscriber('/mobile_base/events/cliff', CliffEvent, cliffCallback)
    rospy.Subscriber('/mobile_base/events/wheel_drop', WheelDropEvent, wheelDropCallback)

    rospy.loginfo("Waiting for /mobile_base/commands/velocity subscribers...")
    stop_cmd = Twist()
    while velocityPub.get_num_connections() == 0 and not rospy.is_shutdown():
        velocityPub.publish(stop_cmd)
        rospy.sleep(0.1)

    rate = rospy.Rate(10)  # 10 ms
    while not rospy.is_shutdown():
        if emergency_brake:  # Emergency brake takes highest priority
            rospy.loginfo("EBRAKE PRESSED")
            command.linear.x = 0.0
            command.angular.z = 0.0
            velocityPub.publish(command)
            rate.sleep()
            continue
        if not received_command:
            command.linear.x = 0.0
            command.angular.z = 0.0
            velocityPub.publish(command)
            rate.sleep()
            continue
        
        if target.linear.x < 0 and leds:
            flashBackwards()
        
        if target.linear.x > 0 and onlyBackwards and backwards:
            command.linear.x = 0
            command.angular.z = 0
        else:
            if smootherMode == 0:
                command.linear.x = target.linear.x
                command.angular.z = target.angular.z
            elif smootherMode == 1:
                if abs(target.linear.x - command.linear.x) > 0.0001:
                    if target.linear.x > command.linear.x:
                        command.linear.x += 0.03
                    else:
                        command.linear.x -= 0.03
                command.angular.z = target.angular.z
            elif smootherMode == 2:
                if abs(target.linear.x - command.linear.x) > 0.0001:
                    if target.linear.x > command.linear.x:
                        command.linear.x += 0.06
                    else:
                        command.linear.x -= 0.06
                command.angular.z = target.angular.z
            changeLEDs(smootherMode)

        velocityPub.publish(command)
        rate.sleep()

if __name__ == '__main__':
    main()
