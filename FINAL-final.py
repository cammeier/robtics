#!/usr/bin/env python3
import tf
import math
import numpy as np
import rospy

import paramiko
import subprocess
import os

import tf2_ros
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from create_msgs.msg import Bumper
from time import time
from std_msgs.msg import Int32
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
is_left_pressed = False
is_right_pressed = False
april_tag_detected = False
global armValue
armValue = 1
cmd_arm_control = None
global expected_child_frames
expected_child_frames = {'tag_1', 'tag_2' , 'tag_3'}  # Set of child frames you're interested in
# Threshold for maximum linear and angular velocities
v_max = 0.04  # Maximum linear velocity (m/s)
w_max = 0.1   # Maximum angular velocity (rad/s)


# Time interval for inactivity (seconds)
INACTIVITY_THRESHOLD = 0.25  # 5 seconds of inactivity
TURN_SPEED = 0.1  # The angular speed for turning

def start_arm_publishing():
    # Create a timer that calls publish_arm every 0.1 seconds (10 Hz)
    timer = rospy.Timer(rospy.Duration(0.1), lambda event: publish_arm())
    return timer

def publish_arm():
    global cmd_arm_control, armValue
    if cmd_arm_control is not None:
        msg = Int32()
        msg.data = armValue  # Assuming armValue is the value you want to publish
        cmd_arm_control.publish(msg)
        rospy.loginfo(f"Published {armValue} to arm control")

def reachPointOdom3(error):
    global rate
    global current_x, current_y, twist_angle, start_x, start_y, start_angle
    rospy.sleep(2)
    start_x = current_x
    start_y = current_y
    start_angle = twist_angleTF

    # Loop to move the robot towards the target position
    while not rospy.is_shutdown():
        # Calculate error from current to destination
        errorArray = cartesian2polar(current_xTF, current_yTF, twist_angleTF)
        rospy.loginfo(errorArray)
        # Control output (velocity and angular velocity)
        vw = compute_vw(errorArray[0], errorArray[1], errorArray[2], 0.15, 0.16, -0.12)

        # Apply the velocity limits
        velocity_publisher(vw[0], vw[1])

        # Stop condition for position error
        if abs(errorArray[0]) < error:
            break

        rate.sleep()


    move_cmd = Twist()
    move_cmd.linear.x = 0
    move_cmd.angular.z = 0
    cmd_vel_pub.publish(move_cmd)



def counter_callbackARM(data):
    global armValue
    armValue = data


def counter_callbackBumper(data):
    global is_left_pressed
    global is_right_pressed
    is_left_pressed = data.is_left_pressed
    is_right_pressed = data.is_right_pressed

def counter_callback(msg):
    global current_x, current_y, twist_angle
    orientation_q = msg.pose.pose.orientation
    euler = tf.transformations.euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
    twist_angle = euler[2]
    current_y = msg.pose.pose.position.y
    current_x = msg.pose.pose.position.x

def counter_callbackTF(msg):
    global current_yTF, current_xTF, current_zTF, twist_angleTF, last_received_timeTF, turn_directionTF, expected_child_frames
    
    # Define expected frames (modify these as needed)
   # expected_parent_frames = {'map', 'odom'}  # Set of parent frames you're interested in

    
    # Update the last received time
    last_received_timeTF = time()
   
    # Iterate through all transforms in the TFMessage
    for transform in msg.transforms:
        # Check if the transform matches expected frames
     #   parent_frame = transform.header.frame_id
        child_frame = transform.child_frame_id

        # Skip transform if it doesn't match expected frames
        if expected_child_frames and child_frame not in expected_child_frames:
           continue
        # Process the transform
        current_xTF = transform.transform.translation.z
        current_yTF = -transform.transform.translation.x

        orientation_q = transform.transform.rotation
        _, _, twist_angleTF = tf.transformations.euler_from_quaternion(
            [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        )

        # Optional: log the processed transform
        rospy.loginfo(f"Processed transform - Parent: Child: {child_frame}")
        rospy.loginfo("april tag: angle = %f", math.degrees(twist_angleTF))

        # Break after processing first matching transform, or remove break to process all
        break


'''
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
        current_yTF = -transform.transform.translation.x
#        current_zTF = transform.transform.translation.x

        orientation_q = transform.transform.rotation
        _, _, twist_angleTF = tf.transformations.euler_from_quaternion(
            [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        )
    rospy.loginfo("april tag: angle = %f", math.degrees(twist_angleTF))

#        turn_directionTF = orientation_q.z
#        twist_angleTF = orientation_q.z
        # Log the values
 #       rospy.loginfo("\nx = %f, y = %f, twist_angle = %f\n", current_xTF, current_yTF, twist_angleTF)
'''

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
#    cmd_vel_pub.publish(move_cmd) 
    
def velocity_publisher(v, w):
    move_cmd = Twist()

    # Apply the velocity clamping
    v, w = clamp_velocity(v, w)
 #   rospy.loginfo("v = %f, w = %f", v, w)
    move_cmd.linear.x = v
    move_cmd.angular.z = w
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
        if is_left_pressed == True or is_right_pressed:
            break
        rate.sleep()
    
    # Stop the robot after reaching the target
    velocity_publisher(0, 0)

    rospy.loginfo("Reached the destination: x = %f, y = %f, angle = %f", current_x, current_y, xythetaArray[2])
    # Turn the robot to face the destination angle
    while abs(xythetaArray[2]) > np.radians(1):

        if math.degrees(xythetaArray[2]) < 0:
           velocity_publisher(0,-0.1)  # Turn robot with angular velocity
           xythetaArray = transformcoordinates(transformed_x, transformed_y, transformed_theta)

        if math.degrees(xythetaArray[2]) > 0:
           velocity_publisher(0,0.1)  # Turn robot with angular velocity
           xythetaArray = transformcoordinates(transformed_x, transformed_y, transformed_theta)


        rospy.loginfo("Reached the destination: x = %f, y = %f, angle = %f", current_x, current_y, xythetaArray[2])
        rate.sleep()
    velocity_publisher(0,0)
    rospy.loginfo("Reached the destination: x = %f, y = %f, angle = %f", destination_x, destination_y, destination_theta)


def read_number_from_file(file_path):
    """
    Reads a number from a .txt file and assigns it to a global variable.

    Parameters:
    - file_path: Full path to the .txt file containing the number.

    Returns:
    - True if the number is successfully read and assigned
    - False if an error occurs
    """
    global armValue  # Access the global variable
    
    try:
        # Open the file and read the content
        with open(file_path, 'r') as file:
            # Read the number as a string, strip any extra whitespace
            number_str = file.read().strip()

            # Try to convert the string to an integer or float, depending on the format of the number
            armValue = int(number_str)  # Use float to handle both integers and floats

            print(f"Number read from file: {armValue}")
            return True

    except FileNotFoundError:
        print(f"Error: The file at {file_path} was not found.")
        return False
    except ValueError:
        print("Error: The file does not contain a valid number.")
        return False
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
        return False



def rsync_transfer(local_path, remote_user, remote_host, remote_path, password):
    """
    Perform an rsync transfer with SSH password authentication.
    
    Parameters:
    - local_path: Full path to the local file or directory to transfer
    - remote_user: Username for the remote host
    - remote_host: IP address or hostname of the remote server
    - remote_path: Full path on the remote server where the file/directory will be copied
    - password: SSH password for authentication
    
    Returns:
    - True if transfer is successful
    - False if transfer fails
    """
    try:
        # First, handle SSH host key verification
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())

        # Attempt to connect and add host key
        try:
            ssh.connect(remote_host, username=remote_user, password=password)
        except paramiko.SSHException as e:
            print(f"SSH Connection Error: {e}")
            return False

        # Close the SSH connection after key verification
        ssh.close()

        # Construct the rsync command with sshpass for password authentication
        rsync_command = [
            'sshpass', 
            '-p', password,
            'rsync', 
            '-avz', 
            '-e', 'ssh',  # Specify SSH as the remote shell
            local_path, 
            f'{remote_user}@{remote_host}:{remote_path}'
        ]

        # Execute the rsync command
        result = subprocess.run(
            rsync_command, 
            check=True,  # Raises CalledProcessError if the command returns a non-zero exit status
            capture_output=True,  # Capture stdout and stderr
            text=True  # Return output as strings instead of bytes
        )

        # Print output for logging/debugging
        print("Rsync transfer successful.")
        print("Standard Output:", result.stdout)

        return True
    
    except subprocess.CalledProcessError as e:
        # Handle rsync command execution errors
        print(f"Rsync transfer failed. Error: {e}")
        print("Standard Error:", e.stderr)
        return False
    except Exception as e:
        # Handle any other unexpected errors
        print(f"An unexpected error occurred: {e}")
        return False



def main():
    global cmd_vel_pubTF, last_received_timeTF, rate
    global cmd_vel_pub
    global cmd_arm_control
    global expected_child_frames
    rospy.sleep(1)
    last_received_time = time()  # Initialize the last received time
    rospy.init_node('odom_sub_node', anonymous=False)
     # Subscriber to the "/odom" topic for current position updates
    rospy.Subscriber("odom", Odometry, counter_callback)
    rospy.Subscriber("/bumper", Bumper, queue_size=14, callback=counter_callbackBumper)

    rate = rospy.Rate(15)  # 15Hz loop rate

    rospy.Subscriber("/tfapril", TFMessage, counter_callbackTF, queue_size=10)
    cmd_vel_pubTF = rospy.Publisher('/visionGuidance/cmd_vel', Twist, queue_size=10)

    cmd_arm_control = rospy.Publisher('/roombaToArm', Int32 , queue_size=10)
    rospy.Subscriber("/ArmToroomba", Int32, queue_size=14, callback=counter_callbackARM) 


    # Publisher for sending velocity commands
    cmd_vel_pub = rospy.Publisher('controller/cmd_vel', Twist, queue_size=10)

#    rospy.sleep(3)
 #   reachPoint()

   # rospy.sleep(3)

#    reachPointOdom(0,0,0)
 #   rospy.sleep(5)
   # reachPointOdom(0,0,90)
        # Compute and apply velocities

    # Stop the robot
    move_cmd = Twist()
    move_cmd.linear.x = 0
    move_cmd.angular.z = 0
    cmd_vel_pub.publish(move_cmd)
    rospy.sleep(1)

#    start_arm_publishing()

    reachPointOdom(.87,0,0)
    reachPointOdom(0,0,-89.5)
    rospy.loginfo("**************************Reached corner 1")
    rospy.loginfo("*********************Persuing Tag 1")
    reachPointOdom3(.32)
    reachPointOdom(0,0,-90)
    rospy.loginfo("$$$$$$$$$$$$$$$$$$Reached corner 2")
    rospy.loginfo("$$$$$$$$$$$$$$$Persuing Tag 2")
    reachPointOdom3(.3)
    reachPointOdom(0,0,86.5)
    rospy.loginfo("$$$$$$$$$$$$$$$$$$$Reached Tag 2/ Midpoint")
    reachPointOdom3(.35)
    reachPointOdom(0,0,85)
    reachPointOdom(.94,0,0)
#    rospy.sleep(2)
   # reachPointOdom(0, 0, twist_angle - 87) #was 90
   # reachPointOdom(0, 0, twist_angle - 87.5) #was 90


#    rospy.sleep(5)
#    SEND FILES AT THIS POINT TO THE ARM

    '''local_file = "/home/pi/finalData/bool.txt"
    remote_user = "da4kv"
    remote_host = "10.14.1.203"
    remote_path = "/home/da4kv/"
    password = "#Roitokoda4"'''


    local_file = "/home/pi/finalData/bool.txt"
    remote_user = "pgk2y"
    remote_host = "10.14.1.201"
    remote_path = "/home/pgk2y/Tortoises"
    password = "SBF8292htj"
    success = rsync_transfer(local_file, remote_user, remote_host, remote_path,password)
    print("Transfer successful" if success else "Transfer failed")

    while armValue == 1:
          rospy.loginfo("waiting for robot to respond")
             # Example usage:
   # Assuming the file 'number.txt' contains a number
          file_path = '/home/pi/finalData/boolArm.txt'  # Replace with the correct file path
          read_number_from_file(file_path)


   # armValue = 0

    if armValue == 0:
       expected_child_frames = {'tag_4', 'tag_5', 'tag_6'}  # Set of child frames you're interested in

       reachPointOdom(0, 0, twist_angle - 86.5) #was 90
       reachPointOdom(0, 0, twist_angle - 87) #was 90
       reachPointOdom(.90, 0, 0)
       reachPointOdom(0, 0, twist_angle - 90)

       rospy.sleep(1)

       reachPointOdom3(1.428)  # set error to certain distance
       reachPointOdom(0, 0, twist_angle - 90)
       reachPointOdom3(.3)
       reachPointOdom(0, 0, twist_angle + 85)
       reachPointOdom3(.3)
       reachPointOdom(0, 0, twist_angle + 87)
       reachPointOdom(.89, 0, twist_angle)






#    reachPointOdom(0,0,90)
#    rospy.loginfo("Reached corner 4")
#    rospy.sleep(1)
#    reachPointOdom(0,0,180)
#    rospy.sleep(1)


    #rospy.spin()

if __name__ == '__main__':
    main()

