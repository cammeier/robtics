#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
class Multiplexer:
    def __init__(self):
        # Initialize variables but don't set up subscribers/publishers here
        self.selected_input = 1  # Start with input 1
        self.last_callback2_time = rospy.get_time()  # Track the time of the last OA callback message
        self.no_oa_timeout = 0.5  # Time to wait before switching back to controller (in seconds)

    def controller_callback(self, data):
        if self.selected_input == 1:
            rospy.loginfo(f"Forwarding from controller: linear=({data.linear.x}, {data.linear.y}, {data.linear.z}); angular=({data.angular.x}, {data.angular.y}, {data.angular.z})")
            self.pub.publish(data)

    def OA_callback(self, data):
        # When data is received from input 2, override input 1 and reset the timeout
        rospy.loginfo(f"Forwarding from obstacle avoidance: linear=({data.linear.x}, {data.linear.y}, {data.linear.z}); angular=({data.angular.x}, {data.angular.y}, {data.angular.z})")
        self.selected_input = 2
        self.pub.publish(data)
        self.last_callback2_time = rospy.get_time()  # Update the time of the last OA message

    def switch_input(self, event):
        # Check if we should switch back to input 1
        current_time = rospy.get_time()
        if self.selected_input == 2 and (current_time - self.last_callback2_time) > self.no_oa_timeout:
            self.selected_input = 1
            rospy.loginfo("Switching back to controller due to OA topic timeout")

        rospy.loginfo(f"Current selected input: {self.selected_input}")

def main():
    rospy.init_node('multiplexer')

    # Instantiate the multiplexer object
    mux = Multiplexer()

    # Set up subscribers and publishers in the main function
    mux.sub1 = rospy.Subscriber("/controller/cmd_vel", Twist, queue_size=10, callback=mux.controller_callback)
    mux.sub2 = rospy.Subscriber("/OA/cmd_vel", Twist, queue_size=10, callback=mux.OA_callback)

    # Set up the publisher
    mux.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # Timer to switch inputs based on a timeout
    rospy.Timer(rospy.Duration(0.1), mux.switch_input)

    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    main()

