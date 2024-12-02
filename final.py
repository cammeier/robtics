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

# Threshold for maximum linear and angular velocities
v_max = 0.04  # Maximum linear velocity (m/s)
w_max = 0.4   # Maximum angular velocity (rad/s)

# Time interval for inactivity (seconds)
INACTIVITY_THRESHOLD = 0.25  # 5 seconds of inactivity
TURN_SPEED = 0.1  # The angular speed for turning

def counter_callback(msg):
    global current_x, current_y, twist_angle
    orientation_q = msg.pose.pose.orientation
    euler = tf.transformations.euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
    twist_angle = euler[2]
    current_y = msg.pose.pose.position.y
    current_x = msg.pose.pose.position.x

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

def velocity_publisher(v, w):
    move_cmd = Twist()

    # Apply the velocity clamping
    v, w = clamp_velocity(v, w)
    rospy.loginfo("v = %f, w = %f", v, w)
    move_cmd.linear.x = v
    move_cmd.angular.z = w
    cmd_vel_pub.publish(move_cmd) 

def reachPoint(destination_x, destination_y, destination_angle):
    global rate
    global current_x, current_y, twist_angle, start_x, start_y, start_angle
    rospy.sleep(2)
    start_x = current_x
    start_y = current_y
    start_angle = twist_angle

    # Destination angle in radians
    destination_theta = np.radians(destination_angle)

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
    
    velocity_publisher(0, 0)
    rospy.loginfo("Final destination: x = %f, y = %f, angle = %f", destination_x, destination_y, destination_angle)

def reachPointOdom(destination_x, destination_y, destination_angle):
    global rate
    rospy.sleep(2)
    start_x = current_x
    start_y = current_y
    start_angle = twist_angle

    destination_theta = np.radians(destination_angle)
    
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
        xythetaArray = transformcoordinates(transformed_x, transformed_y, transformed_theta)
        errorArray = cartesian2polar(xythetaArray[0], xythetaArray[1], xythetaArray[2])

        vw = compute_vw(errorArray[0], errorArray[1], errorArray[2], 0.15, 0.16, -0.12)

        velocity_publisher(vw[0], vw[1])

        if abs(errorArray[0]) < 0.08:
            break

