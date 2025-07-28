#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray

robotpub = rospy.Publisher("/robot_twist", Twist, queue_size=10)
smootherPub = rospy.Publisher("/robot_commands", Int32MultiArray, queue_size=10)#these means we are publishing DOWN to smoother
smoother_com = [1, 1, 1, 0, 1] # by default, bumper, backward, LEDS, emergency brake, smoothing mode

twist = Twist()

def joystickCallback(data):
    global robotpub, smootherPub, twist, smoother_com
    
    
    RT = data.axes[5] # gas peddle
    a_button = data.buttons[0] #go backwards
    left_stick = data.axes[0] #left stick
    

    #new for project 1

    bumper_button= data.buttons[8] #POWER BUTTON
    backward_button= data.buttons[5] # RIGHT BUTTON
    
    LED_button = data.buttons[7] #for led START BUTTON
    smoother_button = data.buttons[3] # for smoother toggle Y
    b_button = data.buttons[1] #b button emergency brake

    
    
    #eco is default 1, 0 is off, 2 is sport
    
    if(bumper_button):
        smoother_com[0] ^= 1
    if(backward_button):
        smoother_com[1] ^= 1
    if(LED_button):
        smoother_com[2] ^= 1
    if(smoother_button==1):
        if(smoother_com[4]>1):
            smoother_com[4]=0
        else:
            smoother_com[4]+=1
            
    if(b_button ==1):
        smoother_com[3] =1
    elif(b_button==0):
        smoother_com[3] = 0
        
    if(a_button == 1):
        if(RT <0.2):
            twist.linear.x = -0.8
        else:
            twist.linear.x = -1+RT
       
        
    else :#a is not pressed
        if(RT < 0.2): #max speed
            twist.linear.x = 0.8
        else:   #a is not pressed plus not pulled down 80%
            twist.linear.x = 1-RT
        
    
    twist.angular.z = left_stick  #left is 1 , right is -1
    
    command_msg = Int32MultiArray()
    command_msg.data = smoother_com

    
    robotpub.publish(twist)#twist
    smootherPub.publish(command_msg)#array
    

def cleanUp():
    global pub, command
    command.linear.x = 0.0
    command.angular.z = 0.0
    pub.publish(command)
    rospy.sleep(1)

def remoteController():
    rospy.init_node("joy_controller", anonymous=True)
    rospy.Subscriber("joy", Joy, joystickCallback)
    
    rospy.on_shutdown(cleanUp)

    rospy.spin()

    

if __name__ == '__main__':
    remoteController()
