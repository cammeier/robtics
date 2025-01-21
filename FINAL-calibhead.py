#!/usr/bin/env python3
# Copyright (C) 2017 Vision-Guided and Intelligent Robotics Lab
# Written by Ali Shafiekhani <Ashafiekhani@mail.missouri.edu>
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation. Meaning:
#         keep this copyright notice,
#         do not try to make money out of it,
#         it's distributed WITHOUT ANY WARRANTY,
#         yada yada yada...
#
# You can get a copy of the GNU General Public License by writing to
# the Free Software Foundation, Inc., 59 Temple Place, Suite 330,
# Boston, MA 02111-1307 USA

import rospy
import tf
from tf import TransformListener
from geometry_msgs.msg import TransformStamped
import math

def main():
    rospy.init_node('head_eye_calibration')

    # Fetch parameters from ROS parameter server
    transX = rospy.get_param('/head_eye_calibration/transX', 0.365)  # transX is distance (robot coordinate) from marker to center of robot
    transY = rospy.get_param('/head_eye_calibration/transY', 0.0)   # transY is lateral distance from center of marker to center of robot
    transZ = rospy.get_param('/head_eye_calibration/transZ', 0.175)  # transZ is height of marker relative to ground
    rollMtoR = rospy.get_param('roll', math.pi / 2)  # roll is rotation around x
    pitchMtoR = rospy.get_param('pitch', 0.0)       # pitch is rotation around y
    yawMtoR = rospy.get_param('yaw', -math.pi / 2)   # yaw is rotation around z

    # Create the transformation from box to robot
    rTb = tf.TransformBroadcaster()
    rTb.setOrigin((transX, transY, transZ))
    rTb.setRotation(tf.transformations.quaternion_from_euler(rollMtoR, pitchMtoR, yawMtoR))

    # Initialize the TF listener
    listener = TransformListener()
    rTc = None
    i = 0

    while not rospy.is_shutdown():
        try:
            # Wait for the transform between 'camera' and 'ar_marker'
            listener.waitForTransform('webcam', 'ar_marker', rospy.Time(0), rospy.Duration(2.0))
            (trans, rot) = listener.lookupTransform('webcam', 'ar_marker', rospy.Time(0))

            # Perform the transformation by combining rTb and the inverse of cTb
            cTb = tf.transformations.compose_matrix(translate=trans, angles=tf.transformations.euler_from_quaternion(rot))
            rTb_matrix = tf.transformations.compose_matrix(translate=(transX, transY, transZ), 
                                                            angles=(rollMtoR, pitchMtoR, yawMtoR))
            rTc = tf.transformations.concatenate_matrices(rTb_matrix, tf.transformations.inverse_matrix(cTb))

            # After 10 iterations, print the transformation result
            if i == 10:
                rollCtoR, pitchCtoR, yawCtoR = tf.transformations.euler_from_matrix(rTc[:3, :3])
                rospy.loginfo("X: {:.3f}, Y: {:.3f}, Z: {:.3f}, | roll: {:.3f}, pitch: {:.3f}, yaw: {:.3f}".format(
                    rTc[0, 3], rTc[1, 3], rTc[2, 3], rollCtoR, pitchCtoR, yawCtoR
                ))
                break

            i += 1
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("Could not get transform")
            rospy.sleep(1.0)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
