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

def reachPoint():
    global last_received_time, current_x, current_y

    # If no tag data is available, try to find a tag (e.g., keep turning in place)
    while current_x is None or current_y is None:
        rospy.loginfo("No tag data received. Turning in place to search for tag.")
        turn_in_place()
        rospy.sleep(1)  # Give some time to rotate
        rospy.loginfo("Trying to find tag...")
        continue  # Keep turning until we get the tag data

    # Main loop to drive towards the target point
    errorArray = cartesian2polar(current_x, current_y, twist_angle)
    vw = compute_vw(errorArray[0], errorArray[1], errorArray[2], .1, .21, -0.29)
    velocity_publisher(vw[0], vw[1])

    while abs(errorArray[0]) > 0.35:
        # Check if we haven't received a new message in a while
        if time() - last_received_time > INACTIVITY_THRESHOLD:
            rospy.loginfo("No new data received, turning in place.")
            turn_in_place()  # Start turning

        else:
            # Update errorArray and compute new velocity commands
            errorArray = cartesian2polar(current_x, current_y, twist_angle)
            vw = compute_vw(errorArray[0], errorArray[1], errorArray[2], .11, .21, 0)
            velocity_publisher(vw[0], vw[1])

        rate.sleep()

    stop_robot()  # Stop when the target is reached

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

def main():
    global cmd_vel_pub, last_received_time, rate

    rospy.sleep(1)
    last_received_time = time()  # Initialize the last received time
    rospy.init_node('odom_sub_node', anonymous=False)

    rate = rospy.Rate(15)  # 15Hz loop rate
    rospy.Subscriber("/tfapril", TFMessage, counter_callback)
    cmd_vel_pub = rospy.Publisher('/visionGuidance/cmd_vel', Twist, queue_size=100)

    rospy.sleep(3)
    reachPoint()
    reachPointOdom(0,0,90)
    reachPointOdom(.93,0,0)
    #rospy.spin()

if __name__ == '__main__':
    main()

