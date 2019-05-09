#!/usr/bin/env python


# Software License Agreement (BSD License)
# This is a ROS node that gets information from a Robotiq 3-finger Gripper and then generates the corresponding haptic feedback to a CyberGrasp. 

#
# Copyright (c) 2012, Robotiq, Inc.
# Revision $Id$


import roslib; roslib.load_manifest('robotiq_s_model_control')
import rospy
from robotiq_s_model_control.msg import _SModel_robot_output  as outputMsg
from robotiq_s_model_control.msg import _SModel_robot_input  as inputMsg
from leap_motion.msg import Human_orion  as LeapMsg_orion # Newly defined LEAP ROS msg
from rain_unity.msg import rain_system  as RainMsg # To receive the system status from Unity
from std_msgs.msg import Float32MultiArray as HapticFeedbackMsg

import sys
import copy
from time import sleep
import numpy as np


global teleoperation_mode, LeapMsg_local, GripperMsg_local, command, pub
teleoperation_mode = ""
LeapMsg_local = None
GripperMsg_local = None

def callback_Mode(status):
    global teleoperation_mode
    teleoperation_mode = status.teleoperation_mode

def callback_Leap(status):
    global LeapMsg_local
    LeapMsg_local = status

def callback_Gripper(status):
    global GripperMsg_local
    GripperMsg_local = status

def generation_hapticfeedback():

    global pub
    global teleoperation_mode, GripperMsg_local


    publish_rate = 250
    rate = rospy.Rate(publish_rate)

    messageData = HapticFeedbackMsg()
    messageData.data = [0]

    while not rospy.is_shutdown():

        messageData.data[0] = 0

        if LeapMsg_local is not None and GripperMsg_local is not None and teleoperation_mode == "MODE_1":
            # Get Current Status
            leap_status = copy.deepcopy(LeapMsg_local)

            if (leap_status.right_hand.is_present is True):
                GripperMsg_local_ = copy.deepcopy(GripperMsg_local)
                # Get the current gripper status
                gPOA_ = GripperMsg_local_.gPOA # Finger Position
                gPRA_ = GripperMsg_local_.gPRA # Finger Requested Position
                PossibleHaptic_ = gPRA_ - gPOA_ # This value will be positive when the gripper touches something while closing. 
                
                if PossibleHaptic_ < 0:
                    PossibleHaptic_ = 0
                
                if PossibleHaptic_ > 0 and GripperMsg_local_.gSTA == 2:
                    PossibleHaptic = PossibleHaptic_
                else:
                    PossibleHaptic = 0


                print("Haptic Feedback: " + str(PossibleHaptic) + "\n")
                messageData.data[0] = PossibleHaptic
            


        pub.publish(messageData)
        rate.sleep()
        



def main():

    global pub
    global command
    
    try:
        rospy.init_node('HapticController', anonymous=True,  disable_signals=True)

        pub = rospy.Publisher('haptic_feedback/', HapticFeedbackMsg, queue_size = 1)

        # Subscribing information from LEAP in Unity
        rospy.Subscriber("/rain/status",RainMsg, callback_Mode, queue_size=1)    
        rospy.Subscriber("/leap_motion/leap_filtered",LeapMsg_orion, callback_Leap, queue_size=1)
        
        # Subscribing information from RobotiQ
        rospy.Subscriber("/SModelRobotInput",inputMsg.SModel_robot_input, callback_Gripper, queue_size=1)

        generation_hapticfeedback()
    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise            




if __name__ == '__main__': 
   
    main()
