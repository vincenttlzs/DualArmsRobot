#!/usr/bin/env python

"""  scene.add_mesh('sheeve', sheeve_pose, filename = '/home/zs/catkin_ws/src/shuang/meshes/tong.STL')
        rospy.sleep(1)
    moveit_obstacles_demo.py - Version 0.1 2014-01-14
    
    Move the gripper to a target pose while avoiding simulated obstacles.
    
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
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import  PlanningScene, ObjectColor, MoveItErrorCodes, MotionPlanDetailedResponse
from geometry_msgs.msg import PoseStamped, Pose
from scipy.spatial.distance import euclidean 
import shlex, subprocess
import os
from copy import deepcopy

GRIPPER_OPEN = [-0.5]
GRIPPER_CLOSED = [0.3]
GRIPPER_NEUTRAL = [0.1]

# Initialize the move group for the right arm
r_arm = MoveGroupCommander('r_arm')
r_gripper = MoveGroupCommander('r_gripper')
l_arm = MoveGroupCommander('l_arm')
l_gripper = MoveGroupCommander('l_gripper')
b_arms = MoveGroupCommander('b_arms')

# Get the name of the end-effector link
end_effector_link = r_arm.get_end_effector_link()
lend_effector_link = l_arm.get_end_effector_link()

# Construct the initial scene object
scene = PlanningSceneInterface()

class MoveItDemo:
    def __init__(self):
        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)
        
        rospy.init_node('moveit_demo')

        rospy.on_shutdown(self.shutdown)                
        
        # Create a scene publisher to push changes to the scene
        self.scene_pub = rospy.Publisher('planning_scene', PlanningScene, queue_size=5)
        
        # Create a dictionary to hold object colors
        self.colors = dict()
        
        # Pause for the scene to get ready
        rospy.sleep(1)                        
           
        # Allow some leeway in position (meters) and orientation (radians)
        r_arm.set_goal_position_tolerance(0.01)
        r_arm.set_goal_orientation_tolerance(0.01)
        l_arm.set_goal_position_tolerance(0.01)
        l_arm.set_goal_orientation_tolerance(0.01)
        b_arms.set_goal_position_tolerance(0.01)
        b_arms.set_goal_orientation_tolerance(0.01)
       
        # Allow replanning to increase the odds of a solution
        r_arm.allow_replanning(True)
        l_arm.allow_replanning(True)
        b_arms.allow_replanning(True)
        # Set the reference frame for pose targets
        reference_frame = 'base_footprint'
        
        # Set the right arm reference frame accordingly
        r_arm.set_pose_reference_frame(reference_frame)
        l_arm.set_pose_reference_frame(reference_frame)
        b_arms.set_pose_reference_frame(reference_frame)
        
        # Allow 5 seconds per planning attempt
        r_arm.set_planning_time(5)
        l_arm.set_planning_time(5)
        b_arms.set_planning_time(5)
        
        # Give each of the scene objects a unique name
        table_id = 'table'
        box1_id = 'box1'
        sheeve_id = 'sheeve'
        bolt_id = 'bolt'
        
        # Remove leftover objects from a previous run
        scene.remove_attached_object(end_effector_link, 'sheeve')
        scene.remove_attached_object(lend_effector_link, 'bolt')
        scene.remove_world_object(table_id)
        scene.remove_world_object(sheeve_id)
        scene.remove_world_object(box1_id)
        scene.remove_world_object(bolt_id)
        
        # Give the scene a chance to catch up
        rospy.sleep(1)

        # Make the table red and the boxes orange
        self.setColor(table_id, 0.8, 0, 0, 1.0)
        self.setColor(box1_id, 0.8, 0.4, 0, 1.0)
        self.setColor(sheeve_id, 0.8, 0.4, 0, 1.0)
        self.setColor(bolt_id, 0.8, 0.4, 0, 1.0)
        # Send the colors to the planning scene
        self.sendColors()    
        
        # Start the arm in the "rest" pose stored in the SRDF file
        r_arm.set_named_target('rest')
        l_arm.set_named_target('l_rest')
        
        b_arms.go()
        
        rospy.sleep(2)
        
        # Set the height of the table off the ground
        table_ground = 0.68
        
        # Set the length, width and height of the table and boxes
        table_size = [0.2, 0.7, 0.02]
        box1_size = [0.1, 0.03, 0.06]
        
        
        # Add a table top and two boxes to the scene
        table_pose = PoseStamped()
        table_pose.header.frame_id = reference_frame
        table_pose.pose.position.x = -0.66
        table_pose.pose.position.y = 0.0
        table_pose.pose.position.z = table_ground + table_size[2] / 2.0
        table_pose.pose.orientation.w = 1.0
        scene.add_box(table_id, table_pose, table_size)
        
        box1_pose = PoseStamped()
        box1_pose.header.frame_id = reference_frame
        box1_pose.pose.position.x = -0.61
        box1_pose.pose.position.y = -0.10
        box1_pose.pose.position.z = table_ground + table_size[2] + box1_size[2] / 2.0
        box1_pose.pose.orientation.w = 1.0   
        scene.add_box(box1_id, box1_pose, box1_size)
        
        sheeve_pose = PoseStamped()
        sheeve_pose.header.frame_id = reference_frame
        sheeve_pose.pose.position.x = -0.661
        sheeve_pose.pose.position.y = 0.21 
        sheeve_pose.pose.position.z = 0.7#table_ground + table_size[2] + 0.2

        sheeve_pose.pose.orientation.x = 0
        sheeve_pose.pose.orientation.y = 0
        sheeve_pose.pose.orientation.z = 0 
        sheeve_pose.pose.orientation.w = 0
        scene.add_mesh('sheeve', sheeve_pose, filename = '/home/zs/catkin_ws/src/shuang/meshes/TONG2.STL', size = (0.0007, 0.0007, 0.0007))
        rospy.sleep(1)
        
        bolt_pose = PoseStamped()
        bolt_pose.header.frame_id = reference_frame
        bolt_pose.pose.position.x = -0.625#-0.6510
        bolt_pose.pose.position.y = -0.05
        bolt_pose.pose.position.z = 0.78 #table_ground + table_size[2] + 0.2

        bolt_pose.pose.orientation.x = 1.57
        bolt_pose.pose.orientation.y = 0
        bolt_pose.pose.orientation.z = 0
        bolt_pose.pose.orientation.w = 0
        scene.add_mesh('bolt', bolt_pose, filename = '/home/zs/catkin_ws/src/shuang/meshes/xiao.STL', size = (0.9, 0.9, 0.9))
        rospy.sleep(1)
        
        target_pose = PoseStamped()
        target_pose.header.frame_id = reference_frame
        target_pose.pose.position.x = sheeve_pose.pose.position.x + 0.15
        target_pose.pose.position.y = sheeve_pose.pose.position.y + 0.041
        target_pose.pose.position.z = sheeve_pose.pose.position.z + 0.06
        target_pose.pose.orientation.x = 0
        target_pose.pose.orientation.y = 0
        target_pose.pose.orientation.z = 0.707
        target_pose.pose.orientation.w = 0.707

        ltarget_pose = PoseStamped()
        ltarget_pose.header.frame_id = reference_frame
        ltarget_pose.pose.position.x = bolt_pose.pose.position.x
        ltarget_pose.pose.position.y = bolt_pose.pose.position.y - 0.17
        ltarget_pose.pose.position.z = bolt_pose.pose.position.z 
        ltarget_pose.pose.orientation.x = 0
        ltarget_pose.pose.orientation.y = 1
        ltarget_pose.pose.orientation.z = 0
        ltarget_pose.pose.orientation.w = 0
        
        # Set the target pose for the arm
        b_arms.set_pose_target(target_pose, end_effector_link)
        b_arms.set_pose_target(ltarget_pose, lend_effector_link)
        b_arms.go() 
      

        l_joint_position = [-1.2809982830897937, -2.091051070837558, -0.6665336612240614, 0.00976332379285824, 1.187592949816742, 6.000794053914294]     
        r_joint_position = [1.0219090942445246, -2.44960502752374, 0.6017259162609786, 2.5680846134999795, 1.3288641740413367, 2.987566090026375]        
        # Set the target pose for the arm
        r_arm.set_joint_value_target(r_joint_position)
        l_arm.set_joint_value_target(l_joint_position)
#####################################################################
        r_arm.go() 
        l_arm.go()
        

        rospy.sleep(2)  

 
#############################################
        
        r_gripper.set_joint_value_target(GRIPPER_OPEN)
       # plan = self.r_gripper.plan()
        #rospy.loginfo(str(plan)
        r_gripper.go()
        rospy.sleep(1)

        l_gripper.set_joint_value_target(GRIPPER_OPEN)
        l_gripper.go()
        rospy.sleep(2)   

###########################################################
        start_pose = r_arm.get_current_pose(end_effector_link).pose
        wpose = deepcopy(start_pose)
        waypoints = []
        waypoints.append(start_pose)
        wpose.position.x -= 0.11
        print "1 part"
        rospy.sleep(1)
        waypoints.append(deepcopy(wpose))
        print "2 part"
        rospy.sleep(1)
        fraction = 0.0
        fraction_trans = fraction
        maxtries = 200
        attempts = 0
            
        # Set the internal state to the current state
        r_arm.set_start_state_to_current_state()
        while fraction < 1.0 and attempts < maxtries:              
            (plan, fraction) = r_arm.compute_cartesian_path (
                                        waypoints,   # waypoint poses
                                        0.01,        # eef_step
                                        0.0,         # jump_threshold
                                        True)        # avoid_collisions
        #save the max value
            if fraction > fraction_trans:
                fraction_trans = fraction
            # Increment the number of attempts 
            attempts += 1               
            # Print out a progress message
            rospy.loginfo("Still trying after " + str(attempts) + " attempts...")
                         
        # If we have a complete plan, execute the trajectory
        fraction = fraction_trans
        if fraction > 0:           
            rospy.loginfo(str(fraction) + " Moving the arm.")   
            r_arm.execute(plan)                          
            rospy.loginfo("Path execution complete.")
        else:
            rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")  


        place_pose = r_arm.get_current_pose(end_effector_link)
        
############left


        print "adjust"
        #Shift the end-effector to the right 5cm
        l_arm.shift_pose_target(1, 0.05, lend_effector_link)
        rospy.sleep(1)
        l_arm.go()
        rospy.sleep(2)
######################################################################################
        print "pick"   
        q = -0.4
        degree = [q]
        r_gripper.set_joint_value_target(degree)
        r_gripper.go()

        scene.attach_mesh(end_effector_link, 'sheeve',sheeve_pose)
        rospy.sleep(1)

        l = 0.1
        ldegree = [l]
        l_gripper.set_joint_value_target(ldegree)
        l_gripper.go()

        scene.attach_mesh(lend_effector_link, 'bolt',bolt_pose)
        rospy.sleep(1)


        print "focus"
   
        r_arm.shift_pose_target(0, 0.10, end_effector_link)
        r_arm.go()
        rospy.sleep(2)

        r_arm.shift_pose_target(2, 0.10, end_effector_link)
        r_arm.go()
        rospy.sleep(2)
        
       
        l_arm.shift_pose_target(2, 0.08, lend_effector_link)
        l_arm.go()
        rospy.sleep(2)
   
        l_arm.shift_pose_target(0, 0.10, lend_effector_link)
        l_arm.go()
        rospy.sleep(2)

        print "focus2"
        current_position = r_arm.get_current_joint_values()
        rospy.loginfo(str(current_position))
        current_position[5] =current_position[5] + 1.57
        rospy.loginfo(str(current_position))
        r_arm.set_joint_value_target(current_position)
        r_arm.go()
        rospy.sleep(1)
       


        print "stick"

        stick_pose = l_arm.get_current_pose(lend_effector_link).pose
        s_wpose = deepcopy(stick_pose)
        s_waypoints = []
        s_waypoints.append(stick_pose)
        s_wpose.position.y += 0.25
        print "1 part"
        rospy.sleep(1)
        s_waypoints.append(deepcopy(s_wpose))
        print "2 part"
        rospy.sleep(1)
        s_fraction = 0.0
        s_maxtries = 200
        s_attempts = 0
            
        # Set the internal state to the current state
        l_arm.set_start_state_to_current_state()
        while s_fraction < 1.0 :#and s_attempts < maxtries:
            (s_plan, s_fraction) = l_arm.compute_cartesian_path (
                                        s_waypoints,   # waypoint poses
                                        0.01,        # eef_step
                                        0.0,         # jump_threshold
                                        True)        # avoid_collisions
                
            # Increment the number of attempts 
            s_attempts += 1               
            # Print out a progress message
            #rospy.loginfo("Still trying after " + str(s_attempts) + " attempts...")
                         
        # If we have a complete plan, execute the trajectory
        
        if s_fraction > 0:           
            rospy.loginfo(str(s_fraction) + " Moving the arm.")   
            r_arm.execute(s_plan)                          
            rospy.loginfo("Path execution complete.")
        else:
            rospy.loginfo("Path planning failed with only " + str(s_fraction) + " success after " + str(s_maxtries) + " attempts.")  
   
#####################################     


        if not plan.joint_trajectory.points:
            print "[ERROR] No trajectory found"
        else:           
          
            #scene.remove_attached_object(lend_effector_link, 'bolt')
            #scene.attach_mesh(end_effector_link, 'bolt')
            rospy.sleep(1) 
            l_gripper.set_joint_value_target(GRIPPER_OPEN)
            l_gripper.go()
            rospy.sleep(2) 


        
        # Set the target pose for the arm
       # r_arm.set_pose_target(place_pose, end_effector_link)
       # r_arm.go() 



            

        # Exit MoveIt cleanly
        moveit_commander.roscpp_shutdown()
        
        # Exit the script        
        moveit_commander.os._exit(0)




















############################################################################################################################################################        
    # Set the color of an object
    def setColor(self, name, r, g, b, a = 0.9):
        # Initialize a MoveIt color object
        color = ObjectColor()
        
        # Set the id to the name given as an argument
        color.id = name
        
        # Set the rgb and alpha values given as input
        color.color.r = r
        color.color.g = g
        color.color.b = b
        color.color.a = a
        
        # Update the global color dictionary
        self.colors[name] = color

    # Actually send the colors to MoveIt!
    def sendColors(self):
        # Initialize a planning scene object
        p = PlanningScene()

        # Need to publish a planning scene diff        
        p.is_diff = True
        
        # Append the colors from the global color dictionary 
        for color in self.colors.values():
            p.object_colors.append(color)
        
        # Publish the scene diff
        self.scene_pub.publish(p)

    def relax_all_servos(self):
        command = 'rosrun rbx2_dynamixels arbotix_relax_all_servos.py'
        args = shlex.split(command)
        subprocess.Popen(args)
           
    def shutdown(self):
        
        # Stop any current arm movement
        r_arm.stop()
        
        # Move to the resting position
        r_arm.set_named_target('rest')
        r_arm.go()
        
        # Relax the servos
       # self.relax_all_servos()
        
        os._exit(0) 

if __name__ == "__main__":
    try:
        MoveItDemo()
    except KeyboardInterrupt:
        raise
    

    
