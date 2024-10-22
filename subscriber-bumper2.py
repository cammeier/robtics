#!/usr/bin/env python3
import rospy
import tf
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from create_msgs.msg import Bumper

def counter_callback(data):
    global is_left_pressed
    global is_right_pressed
    global is_light_left
    global is_light_front_left
    global is_light_center_left
    global is_light_center_right
    global is_light_front_right
    global is_light_right

    is_left_pressed = data.is_left_pressed
    is_right_pressed = data.is_right_pressed
    
    is_light_left = data.is_light_left
    is_light_front_left = data.is_light_front_left
    is_light_center_left = data.is_light_center_left
    is_light_right = data.is_light_right
    is_light_front_right = data.is_light_front_right
    is_light_center_right = data.is_light_center_right
    
    print("callback worked")
    rospy.loginfo("is_left_pressed: %s, is_right_pressed: %s, is_light_left: %s, "
                  "is_light_front_left: %s, is_light_center_left: %s, "
                  "is_light_center_right: %s, is_light_front_right: %s, "
                  "is_light_right: %s", 
                  is_left_pressed, is_right_pressed, is_light_left,
                  is_light_front_left, is_light_center_left,
                  is_light_center_right, is_light_front_right,
                  is_light_right)

def publish_movement(publisher, movement):
    while (is_left_pressed):
        movement.angular.z = -0.1
        print("\npublishing move!!!\n")
        publisher.publish(movement)

    while (is_right_pressed): 
        movement.angular.z = 0.1
        print("\npublishing move!!!\n")
        publisher.publish(movement)

   # movement.angular.z = 0
   # publisher.publish(movement)


if __name__ == '__main__':
    rospy.init_node("OA_Node", anonymous=False)
    rospy.Subscriber("/bumper", Bumper, queue_size=14, callback=counter_callback)
    publisher = rospy.Publisher("/OA/cmd_vel", Twist, queue_size=10)

    print("started")
    movement = Twist() 
    rospy.sleep(1)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        publish_movement(publisher, movement)  # Fixed function call
        rate.sleep()

