#!/usr/bin/env python3
import tf
import math
import numpy as np
import rospy
import tf2_ros
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from time import time

# Global variables for current position and twist angle
current_x = 0.0
current_y = 0.0
twist_angle = 0.0
start_x = 0.0
start_y = 0.0
start_angle = 0.0
cmd_vel_pub = None
cumulative_transform = np.eye(4)
rate = None  # Will be defined in main()


global current_yTF
global current_xTF
global twist_angleTF
global last_received_timeTF
global turn_directionTF
global current_zTF
cmd_vel_pubTF = None



# Threshold for maximum linear and angular velocities
v_max = 0.04  # Maximum linear velocity (m/s)
w_max = 0.4   # Maximum angular velocity (rad/s)

# Time interval for inactivity (seconds)
INACTIVITY_THRESHOLD = 0.25  # 5 seconds of inactivity
TURN_SPEED = 0.1  # The angular speed for turning

def reachPointOdom2():
    global rate
    global current_x, current_y, twist_angle, start_x, start_y, start_angle
    rospy.sleep(2)
    start_x = current_x
    start_y = current_y
    start_angle = twist_angle

    # Destination angle in radians
    destination_theta = np.radians(0)
    
    # Initialize cumulative transformation for the destination
    cumulative_transform = np.array([[math.cos(start_angle), -math.sin(start_angle), 0, start_x],
                                     [math.sin(start_angle), math.cos(start_angle), 0, start_y],
                                     [0, 0, 1, 0],
                                     [0, 0, 0, 1]])
    destination_transform = np.array([
        [math.cos(np.radians(0)), -math.sin(np.radians(0)), 0, (current_xTF-.20)],
        [math.sin(np.radians(0)), math.cos(np.radians(0)), 0, current_yTF+.15],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])


    cumulative_transform = np.matmul(cumulative_transform, destination_transform)

    # Extract transformed position and orientation
    transformed_x = cumulative_transform[0, 3]
    transformed_y = cumulative_transform[1, 3]
    transformed_theta = math.atan2(cumulative_transform[1, 0], cumulative_transform[0, 0])

    # Loop to move the robot towards the target position
    while not rospy.is_shutdown():
        # Calculate error from current to destination
        xythetaArray = transformcoordinates(transformed_x, transformed_y, transformed_theta)
        errorArray = cartesian2polar(xythetaArray[0], xythetaArray[1], xythetaArray[2])

        # Control output (velocity and angular velocity)
        vw = compute_vw(errorArray[0], errorArray[1], errorArray[2], 0.15, 0.16, -0.12)

        # Apply the velocity limits
        velocity_publisher(vw[0], vw[1])

        # Stop condition for position error
        if abs(errorArray[0]) < 0.08:
            break

        rate.sleep()


    move_cmd = Twist()
    move_cmd.linear.x = 0
    move_cmd.angular.z = 0
    cmd_vel_pub.publish(move_cmd)

def counter_callback(msg):
    global current_x, current_y, twist_angle
    orientation_q = msg.pose.pose.orientation
    euler = tf.transformations.euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
    twist_angle = euler[2]
    current_y = msg.pose.pose.position.y
    current_x = msg.pose.pose.position.x

def counter_callbackTF(msg):
    global current_yTF, current_xTF,current_zTF , twist_angleTF, last_received_timeTF, turn_directionTF

    # Update the last received time
    last_received_timeTF = time()

    # Iterate through all transforms in the TFMessage
    for transform in msg.transforms:
#        rospy.loginfo("Received transform:")
#        rospy.loginfo("Translation: x=%f, y=%f, z=%f",
#            transform.transform.translation.x,
#            transform.transform.translation.y,
#            transform.transform.translation.z)        # Access translation values
        current_xTF = transform.transform.translation.z
        current_yTF = transform.transform.translation.y
        current_zTF = transform.transform.translation.x

        orientation_q = transform.transform.rotation
        _, _, twist_angleTF = tf.transformations.euler_from_quaternion(
            [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        )
        turn_directionTF = orientation_q.z

        # Log the values
 #       rospy.loginfo("\nx = %f, y = %f, twist_angle = %f\n", current_xTF, current_yTF, twist_angleTF)


def cartesian2polar(x, y, theta): 
    rho = math.sqrt(x**2 + y**2)
    alpha = -theta + math.atan2(y, x)
    beta = -(theta + alpha)
    return [rho, alpha, beta]

def transformcoordinates(destination_x, destination_y, destination_theta):
    global current_x, current_y, twist_angle, start_x, start_y, start_angle

    dx = current_x
    dy = current_y
    dtheta = twist_angle
  
    RitoR0 = np.array([[math.cos(dtheta), -math.sin(dtheta), 0, dx],
                       [math.sin(dtheta), math.cos(dtheta), 0, dy],
                       [0, 0, 1, 0],
                       [0, 0, 0, 1]])

    ItoR0 = np.array([[math.cos(destination_theta), -math.sin(destination_theta), 0, destination_x],
                      [math.sin(destination_theta), math.cos(destination_theta), 0, destination_y],
                      [0, 0, 1, 0],

                      [0, 0, 0, 1]])

    RitoI = np.matmul(np.linalg.inv(RitoR0), ItoR0)
    delta_x = RitoI[0][3]
    delta_y = RitoI[1][3]
    theta = math.atan2(RitoI[1][0], RitoI[0][0])
    return [delta_x, delta_y, theta]

def compute_vw(pitch, roll, beta, Kpitch, Kroll, Kbeta): 
    K = np.array([[Kpitch, 0, 0],
                  [0, Kroll, Kbeta]]) 
    X = np.array([[pitch], [roll], [beta]])
    vw = np.matmul(K, X)
    return vw

def clamp_velocity(v, w):
    v = min(v, v_max)  # Apply v_max limit
    w = min(w, w_max)  # Apply w_max limit
    return v, w
    

def turn_in_place():
    # Publish a rotation to turn 360 degrees
    move_cmd = Twist()
   # move_cmd.angular.z = 0
   # move_cmd.linear.x = 0
   # cmd_vel_pub.publish(move_cmd)
    #rospy.sleep(1) 
    if turn_directionTF  > 0 : 
       move_cmd.angular.z = TURN_SPEED  # Positive for clockwise, negative for counterclockwise
    if turn_directionTF  < 0 :
       move_cmd.angular.z = -TURN_SPEED  # Positive for clockwise, negative for counterclockwise
    cmd_vel_pub.publish(move_cmd)

def stop_robot():
    # Stop the robot
    move_cmd = Twist()
    move_cmd.linear.x = 0
    move_cmd.angular.z = 0
    cmd_vel_pub.publish(move_cmd)

def velocity_publisherTF(v , w):
    
    move_cmd  = Twist()

#    rospy.loginfo("v = %f u = %f " , v, w )
    move_cmd.linear.x = v
    move_cmd.angular.z = w
    cmd_vel_pub.publish(move_cmd) 
    
def velocity_publisher(v, w):
    move_cmd = Twist()

    # Apply the velocity clamping
    v, w = clamp_velocity(v, w)
    rospy.loginfo("v = %f, w = %f", v, w)
    move_cmd.linear.x = v
    move_cmd.angular.z = w
    cmd_vel_pub.publish(move_cmd) 

def reachPointOdom3():
    global rate
    global current_x, current_y, twist_angle, start_x, start_y, start_angle
    rospy.sleep(2)
    start_x = current_x
    start_y = current_y
    start_angle = twist_angle

    # Destination angle in radians
    destination_theta = np.radians(0)
    
    # Initialize cumulative transformation for the destination
    cumulative_transform = np.array([[math.cos(start_angle), -math.sin(start_angle), 0, start_x],
                                     [math.sin(start_angle), math.cos(start_angle), 0, start_y],
                                     [0, 0, 1, 0],
                                     [0, 0, 0, 1]])
    destination_transform = np.array([
        [math.cos(np.radians(0)), -math.sin(np.radians(0)), 0, (current_xTF-.20)],
        [math.sin(np.radians(0)), math.cos(np.radians(0)), 0, current_yTF],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])


    cumulative_transform = np.matmul(cumulative_transform, destination_transform)

    # Extract transformed position and orientation
    transformed_x = cumulative_transform[0, 3]
    transformed_y = cumulative_transform[1, 3]
    transformed_theta = math.atan2(cumulative_transform[1, 0], cumulative_transform[0, 0])

    # Loop to move the robot towards the target position
    while not rospy.is_shutdown():
        # Calculate error from current to destination
        xythetaArray = transformcoordinates(transformed_x, transformed_y, transformed_theta)
        errorArray = cartesian2polar(xythetaArray[0], xythetaArray[1], xythetaArray[2])

        # Control output (velocity and angular velocity)
        vw = compute_vw(errorArray[0], errorArray[1], errorArray[2], 0.15, 0.16, -0.12)

        # Apply the velocity limits
        velocity_publisher(vw[0], vw[1])

        # Stop condition for position error
        if abs(errorArray[0]) < 0.04:
            break

        rate.sleep()


    move_cmd = Twist()
    move_cmd.linear.x = 0
    move_cmd.angular.z = 0
    cmd_vel_pub.publish(move_cmd)
def reachPointOdom(destination_x, destination_y, destination_angle):
    global rate
    global current_x, current_y, twist_angle, start_x, start_y, start_angle
    rospy.sleep(2)
    start_x = current_x
    start_y = current_y
    start_angle = twist_angle

    # Destination angle in radians
    destination_theta = np.radians(destination_angle)
    
    # Initialize cumulative transformation for the destination
    cumulative_transform = np.array([[math.cos(start_angle), -math.sin(start_angle), 0, start_x],
                                     [math.sin(start_angle), math.cos(start_angle), 0, start_y],
                                     [0, 0, 1, 0],
                                     [0, 0, 0, 1]])
    destination_transform = np.array([
        [math.cos(destination_theta), -math.sin(destination_theta), 0, destination_x],
        [math.sin(destination_theta), math.cos(destination_theta), 0, destination_y],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

    cumulative_transform = np.matmul(cumulative_transform, destination_transform)

    # Extract transformed position and orientation
    transformed_x = cumulative_transform[0, 3]
    transformed_y = cumulative_transform[1, 3]
    transformed_theta = math.atan2(cumulative_transform[1, 0], cumulative_transform[0, 0])

    # Loop to move the robot towards the target position
    while not rospy.is_shutdown():
        # Calculate error from current to destination
        xythetaArray = transformcoordinates(transformed_x, transformed_y, transformed_theta)
        errorArray = cartesian2polar(xythetaArray[0], xythetaArray[1], xythetaArray[2])

        # Control output (velocity and angular velocity)
        vw = compute_vw(errorArray[0], errorArray[1], errorArray[2], 0.15, 0.16, -0.12)

        # Apply the velocity limits
        velocity_publisher(vw[0], vw[1])

        # Stop condition for position error
        if abs(errorArray[0]) < 0.08:
            break

        rate.sleep()
    
    # Stop the robot after reaching the target
    velocity_publisher(0, 0)

    rospy.loginfo("Reached the destination: x = %f, y = %f, angle = %f", current_x, current_y, xythetaArray[2])
    # Turn the robot to face the destination angle
    while abs(xythetaArray[2]) > np.radians(1): 
        velocity_publisher(0, 0.1)  # Turn robot with angular velocity
        xythetaArray = transformcoordinates(transformed_x, transformed_y, transformed_theta)

        rospy.loginfo("Reached the destination: x = %f, y = %f, angle = %f", current_x, current_y, xythetaArray[2])
        rate.sleep()
    velocity_publisher(0,0)
    rospy.loginfo("Reached the destination: x = %f, y = %f, angle = %f", destination_x, destination_y, destination_theta)

def main():
    global cmd_vel_pubTF, last_received_timeTF, rate
    global cmd_vel_pub

    rospy.sleep(1)
    last_received_time = time()  # Initialize the last received time
    rospy.init_node('odom_sub_node', anonymous=False)
     # Subscriber to the "/odom" topic for current position updates
    rospy.Subscriber("odom", Odometry, counter_callback)


    rate = rospy.Rate(15)  # 15Hz loop rate

    rospy.Subscriber("/tfapril", TFMessage, counter_callbackTF)
    cmd_vel_pubTF = rospy.Publisher('/visionGuidance/cmd_vel', Twist, queue_size=1)



    # Publisher for sending velocity commands
    cmd_vel_pub = rospy.Publisher('controller/cmd_vel', Twist, queue_size=100)

#    rospy.sleep(3)
 #   reachPoint()

   # rospy.sleep(3)

#    reachPointOdom(0,0,0)
 #   rospy.sleep(5)
   # reachPointOdom(0,0,90)
    rospy.sleep(1)
    reachPointOdom(.87,0,0)
    reachPointOdom(0,0,-90.5)
    rospy.loginfo("**************************Reached corner 1")

    rospy.sleep(1)
    rospy.loginfo("*********************Persuing Tag 1")
    reachPointOdom2()
  #  rospy.sleep(3)

    reachPointOdom(0,0,-85.5)
    rospy.loginfo("$$$$$$$$$$$$$$$$$$Reached corner 2")
 #   reachPointOdom(0,0,-85)


   # rospy.sleep(5)
    rospy.loginfo("$$$$$$$$$$$$$$$Persuing Tag 2")

    reachPointOdom2()
    reachPointOdom(0,0,86.5)
    rospy.loginfo("$$$$$$$$$$$$$$$$$$$Reached Tag 2/ Midpoint")
    reachPointOdom2()



#    reachPointOdom2()





    rospy.sleep(1)
#    rospy.loginfo("Persuing Tag 3")
#    reachPoint()
    reachPointOdom(0,0,90)
    rospy.loginfo("Reached corner 3")
#    rospy.sleep(1)
    reachPointOdom(.79,0,0)
#    SEND FILES AT THIS POINT TO THE ARM


#    reachPointOdom(0,0,90)
#    rospy.loginfo("Reached corner 4")
#    rospy.sleep(1)
#    reachPointOdom(0,0,180)
#    rospy.sleep(1)


    #rospy.spin()

if __name__ == '__main__':
    main()
