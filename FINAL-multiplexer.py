#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry

class Multiplexer:
    def __init__(self):
        # Initialize variables
        self.selected_input = 2  # Start with Vision Guidance as default
        self.last_oa_time = rospy.get_time()  # Track the time of the last OA callback message
        self.last_tfapril_time = rospy.get_time()  # Track the time of the last /tfapril message
        self.last_controller_time = rospy.get_time()  # Track the time of the last controller message
        self.no_oa_timeout = 0.5  # Time to wait before switching from OA to Vision (in seconds)
        self.no_tfapril_timeout = 15  # Timeout for /tfapril before switching to Controller (in seconds)
    
    def oa_callback(self, data):
        # When data is received from OA, update last time and switch to OA
        rospy.loginfo(f"Forwarding from OA: linear=({data.linear.x}, {data.linear.y}, {data.linear.z}); angular=({data.angular.x}, {data.angular.y}, {data.angular.z})")
        self.selected_input = 1  # Set OA as the active input
        self.pub.publish(data)
        self.last_oa_time = rospy.get_time()  # Update the time of the last OA message

    def tfapril_callback(self, msg):
        # When data is received from /tfapril, update last time
        self.last_tfapril_time = rospy.get_time()  # Update the time of the last /tfapril message
        rospy.loginfo(f"Received /tfapril message at {self.last_tfapril_time}")

    def controller_callback(self, data):
        # When data is received from Controller, update last time and switch to Controller
        rospy.loginfo(f"Forwarding from Controller: linear=({data.linear.x}, {data.linear.y}, {data.linear.z}); angular=({data.angular.x}, {data.angular.y}, {data.angular.z})")
        self.selected_input = 3  # Set Controller as the active input
        self.pub.publish(data)
        self.last_controller_time = rospy.get_time()  # Update the time of the last Controller message

    def vision_callback(self, data):
        # When data is received from Vision, update last time
        rospy.loginfo(f"Forwarding from Vision: linear=({data.linear.x}, {data.linear.y}, {data.linear.z}); angular=({data.angular.x}, {data.angular.y}, {data.angular.z})")
        self.selected_input = 2  # Set Vision as the active input
        self.pub.publish(data)

    def switch_input(self, event):
        # Check the time of the last message for each input and switch if needed
        current_time = rospy.get_time()

        # Check if OA is still active (Highest Priority)
        if current_time - self.last_oa_time <= self.no_oa_timeout:
            if self.selected_input != 1:  # If we are not already in OA state, switch to OA
                self.selected_input = 1  # Switch to OA
                rospy.loginfo("**********Switching to OA due to new OA message*************")
            return  # Return early if we're switching to OA

        # Check if Controller should be active (Second Priority)
        # If /tfapril has been inactive for 15 seconds, switch to Controller
        if current_time - self.last_tfapril_time > self.no_tfapril_timeout:
            if self.selected_input != 3:  # If not already in Controller
                self.selected_input = 3  # Switch to Controller
                rospy.loginfo("Switching to Controller due to /tfapril timeout")
            return  # Return early if we're switching to Controller

        # If none of the above conditions are met, default to Vision Guidance
        if self.selected_input != 2:
            self.selected_input = 2  # Switch to Vision
            rospy.loginfo("Defaulting to Vision Guidance")

        rospy.loginfo(f"Current selected input: {self.selected_input}")

    def publish_selected_input(self):
        # Publish the correct data based on the selected input
        if self.selected_input == 1:
            rospy.loginfo("Publishing from OA")
            self.oa_data = Twist()  # Replace with actual OA data
            self.pub.publish(self.oa_data)
        elif self.selected_input == 2:
            rospy.loginfo("Publishing from Vision")
            self.vision_data = Twist()  # Replace with actual Vision data
            self.pub.publish(self.vision_data)
        elif self.selected_input == 3:
            rospy.loginfo("Publishing from Controller")
            self.controller_data = Twist()  # Replace with actual Controller data
            self.pub.publish(self.controller_data)

def main():
    rospy.init_node('multiplexerAll')

    # Instantiate the multiplexer object
    mux = Multiplexer()

    # Set up subscribers for each input
    mux.sub1 = rospy.Subscriber("/OA/cmd_vel", Twist, queue_size=10, callback=mux.oa_callback)
    mux.sub2 = rospy.Subscriber("/visionGuidance/cmd_vel", Twist, queue_size=10, callback=mux.vision_callback)
    mux.sub3 = rospy.Subscriber("/controller/cmd_vel", Twist, queue_size=10, callback=mux.controller_callback)
    mux.sub4 = rospy.Subscriber("/tfapril", TFMessage, queue_size=10, callback=mux.tfapril_callback)  # Add subscriber for /tfapril

    # Set up the publisher
    mux.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # Timer to switch inputs based on timeouts
    rospy.Timer(rospy.Duration(0.1), mux.switch_input)  # Check every 0.1s to see if timeout occurred

    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    main()
