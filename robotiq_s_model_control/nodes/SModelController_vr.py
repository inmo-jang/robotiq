#!/usr/bin/env python


# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Robotiq, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Robotiq, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Copyright (c) 2012, Robotiq, Inc.
# Revision $Id$

"""@package docstring
Command-line interface for sending simple commands to a ROS node controlling a S-Model gripper.

This serves as an example for publishing messages on the 'SModelRobotOutput' topic using the 'SModel_robot_output' msg type for sending commands to a S-Model gripper. In this example, only the simple control mode is implemented. For using the advanced control mode, please refer to the Robotiq support website (support.robotiq.com).
"""

import roslib; roslib.load_manifest('robotiq_s_model_control')
import rospy
from robotiq_s_model_control.msg import _SModel_robot_output  as outputMsg
from leap_motion.msg import Human_orion  as LeapMsg_orion # Newly defined LEAP ROS msg
from rain_unity.msg import rain_system  as RainMsg # To receive the system status from Unity

import sys
from time import sleep



def callback_MODE(status):
    global teleoperation_mode
    teleoperation_mode = status.teleoperation_mode

def callback_Gripper(status):

    global pub
    global teleoperation_mode
    global command


    if(teleoperation_mode == "MODE_1"):
        if (status.left_hand.is_present):
            # Control the Gripper
            grab_strength_from_unity = status.left_hand.grab_strength
            if (grab_strength_from_unity <= 1 and grab_strength_from_unity > 0.5): # Fully Close
                # grab_strength = 255 # This is for the on-off controller
                grab_strength = round(255*(grab_strength_from_unity-0.5)*2,0) # Give an integer
            else: # Fully Open
                grab_strength = 0 # Fully Open
                

            command.rPRA = grab_strength

            pub.publish(command)

            rospy.sleep(0.1)


def listener():
    # Initialise "teleoperation_mode", otherwise callback error comes out from "callback_Gripper"
    global teleoperation_mode
    teleoperation_mode = ""

    # Subscribing information from LEAP in Unity
    rospy.Subscriber("/rain/status",RainMsg, callback_MODE, queue_size=1)    
    rospy.Subscriber("/leap_motion/leap_filtered",LeapMsg_orion, callback_Gripper, queue_size=1)
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


def publisher(myArg1):
    global pub
    global command
    
    if (myArg1 == 'gazebo') or (myArg1 == 'real'):
        try:
            rospy.init_node('Controller_Gripper', anonymous=True,  disable_signals=True)
    
            if myArg1 == 'gazebo':
                ## This is for the gazebo Gripper:
                pub = rospy.Publisher('right_hand/command', outputMsg.SModel_robot_output, queue_size = 1)

            elif myArg1 == 'real':
                ## This is for the real Gripper:
                pub = rospy.Publisher('SModelRobotOutput', outputMsg.SModel_robot_output)
  
            # Initilise the gripper
            command = outputMsg.SModel_robot_output();
    
            command.rACT = 1 # Activation
            command.rGTO = 1 # 
            command.rSPA = 255 # Opening/Closing Speed (Maximum = 255)
            command.rICF = 0 # Individual Control Finger mode (1: Yes / 0: No)
            command.rFRA = 0 # Final grasping force (Maximum = 255). This does not matter in Gazebo.

         
            pub.publish(command)
            rospy.sleep(0.1)

        except KeyboardInterrupt:
            rospy.signal_shutdown("KeyboardInterrupt")
            raise            
    else:
        rospy.signal_shutdown("Wrong Argument: It should be either 'gazebo' or 'real'")


def main(myArg1):
    publisher(myArg1)
    listener()

if __name__ == '__main__': 
    # This is for real
    if len(sys.argv) < 2:
        print("Usage: SModelController.py [gazebo] or [real]")
    else:
        main(sys.argv[1])
        
    # This is for debug    
    # main('gazebo')