#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist 
tele_x = 0
tele_z = 0
pub_tele = False

OA_x = 0
OA_z = 0
pub_OA = False

cont_x = 0
cont_z = 0
pub_cont = False 

def tele(msg):
    global tele_x
    global tele_z
    global pub_tele
    
    tele_x = msg.linear.x
    tele_z = msg.angular.z
    pub_tele = True
    return

def OA(msg):
    global OA_x
    global OA_z
    global pub_OA
    
    OA_x = msg.linear.x
    OA_z = msg.angular.z
    pub_OA = True
    
    return

def controller(msg):
    global cont_x
    global cont_z
    global pub_cont
    
    cont_x = msg.linear.x
    cont_z = msg.angular.z
    pub_cont = True 
    
    return

#this choose which message to publish with the priority going tele,OA,controller
def publish(pub,motion):
    global tele_x
    global tele_z
    global pub_tele
    global OA_x
    global OA_z 
    global pub_OA
    global cont_x
    global cont_z
    global pub_cont
    
    if pub_tele == True:
        motion.linear.x = tele_x
        motion.angular.z = tele_z
        pub.publish(motion)
        pub_tele = False

    elif pub_OA == True:
        motion.linear.x = OA_x
        motion.angular.z = OA_z
        pub.publish(motion)
        pub_OA = False


    elif pub_cont == True:
        motion.linear.x = cont_x
        motion.angular.z = cont_z
        pub.publish(motion)
        pub_cont = False


    return

if __name__ == '__main__':
    try:
        rospy.init_node("OA_node" , anonymous=True)
        rospy.Subscriber("/controller/cmd_vel",Twist,queue_size=100,callback=controller)
        rospy.Subscriber("/OA/cmd_vel",Twist,queue_size=100,callback=OA)
        rospy.Subscriber("/tele/cmd_vel",Twist,queue_size=100,callback=tele)
        pub = rospy.Publisher("/cmd_vel",Twist,queue_size=10)

        motion = Twist()

        rospy.sleep(1)
        r = rospy.Rate(10)

        while not rospy.is_shutdown():
            publish(pub,motion)
            r.sleep()
        
    except rospy.ROSInterruptException:
        pass