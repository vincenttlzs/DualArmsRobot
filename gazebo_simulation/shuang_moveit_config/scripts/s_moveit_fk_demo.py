#!/usr/bin/env python

"""
    moveit_fk_demo.py - Version 0.1 2014-01-14
    
    Use forward kinemtatics to move the arm to a specified set of joint angles
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2014 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
"""

import rospy, sys
import moveit_commander
from control_msgs.msg import GripperCommand

class MoveItDemo:
    def __init__(self):
        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)

        # Initialize the ROS node
        rospy.init_node('s_moveit_demo', anonymous=True)
        
        GRIPPER_OPEN = [-0.5]
        GRIPPER_CLOSED = [0.2]
        GRIPPER_NEUTRAL = [0.0]
 
        # Connect to the right_arm move group
        r_arm = moveit_commander.MoveGroupCommander('r_arm')
        
        # Connect to the right_gripper move group
        r_gripper = moveit_commander.MoveGroupCommander('r_gripper')
                
        # Get the name of the end-effector link
        end_effector_link = r_arm.get_end_effector_link()
        
        # Display the name of the end_effector link
        rospy.loginfo("The end effector link is: " + str(end_effector_link))
        
        # Set a small tolerance on joint angles
        r_arm.set_goal_joint_tolerance(0.001)
        r_gripper.set_goal_joint_tolerance(0.001)
        
        # Start the arm target in "rest pose stored in the SRDF file
        r_arm.set_named_target('rest')
        
        # Plan a trajectory to the goal configuration
        traj = r_arm.plan()
         
        # Execute the planned trajectory
        r_arm.execute(traj)
        
        # Pause for a moment
        rospy.sleep(1)
         
        # Set the gripper target to neutal position using a joint value target
        r_gripper.set_joint_value_target(GRIPPER_NEUTRAL)
         
        # Plan and execute the gripper motion
        r_gripper.go()
        rospy.sleep(1)
         
        # Set target joint values for the arm: joints are in the order they appear in
        # the kinematic tree.
        joint_positions = [1.45, -1.4, -1.2, 0, 0, 0]
 
        # Set the arm's goal configuration to the be the joint positions
        r_arm.set_joint_value_target(joint_positions)
                 
        # Plan and execute the motion
        r_arm.go()
        rospy.sleep(1)
         
        # Save this configuration for later
        r_arm.remember_joint_values('saved_config', joint_positions)
         
        # Close the gripper as if picking something up
        r_gripper.set_joint_value_target(GRIPPER_CLOSED)
        r_gripper.go()
        rospy.sleep(1)
                 
        # Set the arm target to the named "straight_out" pose stored in the SRDF file
        r_arm.set_named_target('straight')
         
        # Plan and execute the motion
        r_arm.go()
        rospy.sleep(1)
                  
        # Set the goal configuration to the named configuration saved earlier
        r_arm.set_named_target('saved_config')
         
        # Plan and execute the motion
        r_arm.go()
        rospy.sleep(1)
         
        # Open the gripper as if letting something go
        r_gripper.set_joint_value_target(GRIPPER_OPEN)
        r_gripper.go()
        rospy.sleep(1)
         
        # Return the arm to the named "resting" pose stored in the SRDF file
        r_arm.set_named_target('rest')
        r_arm.go()
        rospy.sleep(1)
         
        # Return the gripper target to neutral position
        r_gripper.set_joint_value_target(GRIPPER_NEUTRAL)
        r_gripper.go()
        rospy.sleep(1)
        
        # Cleanly shut down MoveIt
        moveit_commander.roscpp_shutdown()
        
        # Exit the script
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    try:
        MoveItDemo()
    except rospy.ROSInterruptException:
        pass
