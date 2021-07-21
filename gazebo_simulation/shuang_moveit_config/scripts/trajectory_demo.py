#!/usr/bin/env python

"""
    trajectory_demo.py - Version 0.1 2014-01-14
    
    Send a trajectory to the FollowJointTrajectoryAction server
    
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

import rospy
import actionlib

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class TrajectoryDemo():
    def __init__(self):
        rospy.init_node('trajectory_demo')
        
        # Set to True to move back to the starting configurations
        reset = rospy.get_param('~reset', False)
        
        # Set to False to wait for arm to finish before moving head
        sync = rospy.get_param('~sync', True)
        
        # Which joints define the arm?
        l_arm_joints = ['base_l_1_joint',
                      'l_1_l_2_joint',
                      'l_2_l_3_joint', 
                      'l_3_l_4_joint',
                      'l_4_l_5_joint',
                      'l_arm_wrist_flex_joint',
                      'l_gripper_finger_joint']
        
        # Which joints define the head?
        r_arm_joints = ['base_r_1_joint',
                      'r_1_r_2_joint',
                      'r_2_r_3_joint', 
                      'r_3_r_4_joint',
                      'r_4_r_5_joint',
                      'r_arm_wrist_flex_joint',
                      'r_gripper_finger_joint']
        
        if reset:
            # Set the arm back to the resting position
            l_arm_goal  = [1, 1, 1, 1, 1, 1, 1]
            
            # Re-center the head
            r_arm_goal  = [1, 0, 0, 0, 0, 0, 0]  
        else:
            # Set a goal configuration for the arm
            l_arm_goal  = [-0.3, -0.01, -0.01, 0, 0, -0.1, 0.1]
            
            # Set a goal configuration for the head
            r_arm_goal  = [-0.3, -2.0, -1.0, 0.8, 1.0, -0.7, 0.1]
    
        # Connect to the right arm trajectory action server
        rospy.loginfo('Waiting for right arm trajectory controller...')
        
        l_arm_client = actionlib.SimpleActionClient('l_arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        
        l_arm_client.wait_for_server()
        
        rospy.loginfo('...connected.')
        
        # Connect to the head trajectory action server
        rospy.loginfo('Waiting for head trajectory controller...')
    
        r_arm_client = actionlib.SimpleActionClient('r_arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
       
        r_arm_client.wait_for_server()
        
        rospy.loginfo('...connected.')    
    
        # Create a single-point arm trajectory with the arm_goal as the end-point
        l_arm_trajectory = JointTrajectory()
        l_arm_trajectory.joint_names = l_arm_joints
        l_arm_trajectory.points.append(JointTrajectoryPoint())
        l_arm_trajectory.points[0].positions = l_arm_goal
        l_arm_trajectory.points[0].velocities = [0.0 for i in l_arm_joints]
        l_arm_trajectory.points[0].accelerations = [0.0 for i in l_arm_joints]
        l_arm_trajectory.points[0].time_from_start = rospy.Duration(3.0)
    
        # Send the trajectory to the arm action server
        rospy.loginfo('Moving the arm to goal position...')
        
        # Create an empty trajectory goal
        l_arm_goal = FollowJointTrajectoryGoal()
        
        # Set the trajectory component to the goal trajectory created above
        l_arm_goal.trajectory = l_arm_trajectory
        
        # Specify zero tolerance for the execution time
        l_arm_goal.goal_time_tolerance = rospy.Duration(0.0)
    
        # Send the goal to the action server
        l_arm_client.send_goal(l_arm_goal)
        
        if not sync:
            # Wait for up to 5 seconds for the motion to complete 
            l_arm_client.wait_for_result(rospy.Duration(5.0))
        
        # Create a single-point head trajectory with the head_goal as the end-point
        r_arm_trajectory = JointTrajectory()
        r_arm_trajectory.joint_names = r_arm_joints
        r_arm_trajectory.points.append(JointTrajectoryPoint())
        r_arm_trajectory.points[0].positions = r_arm_goal
        r_arm_trajectory.points[0].velocities = [0.0 for i in r_arm_joints]
        r_arm_trajectory.points[0].accelerations = [0.0 for i in r_arm_joints]
        r_arm_trajectory.points[0].time_from_start = rospy.Duration(3.0)
    
        # Send the trajectory to the head action server
        rospy.loginfo('Moving the head to goal position...')
        
        r_arm_goal = FollowJointTrajectoryGoal()
        r_arm_goal.trajectory = r_arm_trajectory
        r_arm_goal.goal_time_tolerance = rospy.Duration(0.0)
    
        # Send the goal
        r_arm_client.send_goal(r_arm_goal)
        
        # Wait for up to 5 seconds for the motion to complete 
        r_arm_client.wait_for_result(rospy.Duration(5.0))
        
        rospy.loginfo('...done')
        
if __name__ == '__main__':
    try:
        TrajectoryDemo()
    except rospy.ROSInterruptException:
        pass
    
