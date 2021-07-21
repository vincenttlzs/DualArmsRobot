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
        
        c_position = b_arms.get_current_joint_values()
        rospy.loginfo(str(c_position))

        # Start the arm in the "rest" pose stored in the SRDF file
        r_arm.set_named_target('rest')
        l_arm.set_named_target('l_rest')
        
        l_arm.go()
        
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
        bolt_pose.pose.position.x = -0.63
        bolt_pose.pose.position.y = -0.03
        bolt_pose.pose.position.z = 0.78 #table_ground + table_size[2] + 0.2

        bolt_pose.pose.orientation.x = 1.57
        bolt_pose.pose.orientation.y = 0
        bolt_pose.pose.orientation.z = 0
        bolt_pose.pose.orientation.w = 0
        scene.add_mesh('bolt', bolt_pose, filename = '/home/zs/catkin_ws/src/shuang/meshes/xiao.STL', size = (0.85, 0.85, 0.85))
        rospy.sleep(1)
        


        # Set the target pose for the arm
        joint_position = [-1.3856071517624684, -2.1465670384371442, -0.5589531813195253, -0.02179324588034455, 1.1174736958415603, -6.503100949354163, 1.1863131914075702, -2.4780686453560463, 0.6746025014989107, -0.3815969838367581, -1.3311865216541436, 6.198178272512462]         
        b_arms.set_joint_value_target(joint_position)       
        b_arms.go()
        rospy.sleep(2) 
#    tiao shi     joint_position = [4.8306481586769223e-05, 1.1738484395392832, -2.2499690635839475, -2.4360941311922617, -8.593123429454863e-05, 0.6923971168090738, -2.1108083752915263e-05, 2.7348946366581797, -4.839858338236809e-05, 1.4466913641419177, -6.811968237161637e-05, -0.49990119480709544, 9.400287573599014, -0.4999259925455786]         
 #       b_arms.set_joint_value_target(joint_position)       
 #       b_arms.go()
 #       rospy.sleep(2) 
#    tiao shi     joint_position = [4.8306481586769223e-05, -2.2499690635839475, -8.593123429454863e-05, -2.1108083752915263e-05, -4.839858338236809e-05, 0.6923971168090738, 1.1738484395392832, -2.4360941311922617, 0.6923971168090738, -4.839858338236809e-05, -6.811968237161637e-05, -0.49990119480709544, 9.400287573599014, -0.4999259925455786]         
 #       b_arms.set_joint_value_target(joint_position)       
 #       b_arms.go()
 #       rospy.sleep(2)

        # Set the target pose for the arm
       # joint_position = [-1.2810749444864722, -2.0910708724019793, -0.6664769485253113, 0.009723949887180892, 1.1875087936566389, 6.000757700510079, 1.0219999064482783, -2.4495826469963986, 0.6016401158337337, 2.5680025154927772, 1.3288638145246061, 2.987592838251328]         
       # b_arms.set_joint_value_target(joint_position)       
      #  b_arms.go()
      #  rospy.sleep(2) 



        # open the gripper
        r_gripper.set_joint_value_target(GRIPPER_OPEN)
        r_gripper.go()
        rospy.sleep(1)
        l_gripper.set_joint_value_target(GRIPPER_OPEN)
        l_gripper.go()
        rospy.sleep(2)   

        #move straight line by cartesian path
        print "cartesian path"
        start_pose = r_arm.get_current_pose(end_effector_link).pose
        wpose = deepcopy(start_pose)
        waypoints = []
        waypoints.append(start_pose)
        wpose.position.x -= 0.11

        rospy.sleep(1)
        waypoints.append(deepcopy(wpose))
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

        #set the place pose that would be use latter
        place_pose = r_arm.get_current_pose(end_effector_link)

        #move the left arm
        print "adjust"
        #Shift the end-effector to the right 5cm
        l_arm.shift_pose_target(1, 0.05, lend_effector_link)
        rospy.sleep(1)
        l_arm.go()
        rospy.sleep(2)
 
        #catch the object
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

        #go to a right pose for motion
        print "adjust"   
#        r_arm.shift_pose_target(0, 0.11, end_effector_link)
#        r_arm.go()
#        rospy.sleep(2)

        print "cartesian path"
        start_pose = r_arm.get_current_pose(end_effector_link).pose
        wpose = deepcopy(start_pose)
        waypoints = []
        waypoints.append(start_pose)
        wpose.position.x += 0.11

        rospy.sleep(1)
        waypoints.append(deepcopy(wpose))
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


        r_arm.shift_pose_target(2, 0.138, end_effector_link)
        r_arm.go()
        rospy.sleep(2)       
     
        l_arm.shift_pose_target(2, 0.095, lend_effector_link)
        l_arm.go()
        rospy.sleep(2)
   
        l_arm.shift_pose_target(0, 0.105, lend_effector_link)
        l_arm.go()
        rospy.sleep(2)
 
        b_position = b_arms.get_current_joint_values()
        rospy.loginfo(str(b_position))

        print "rotate"
        current_position = r_arm.get_current_joint_values()
        rospy.loginfo(str(current_position))
        current_position[5] =current_position[5] + 1.57
        rospy.loginfo(str(current_position))
        r_arm.set_joint_value_target(current_position)
        r_arm.go()
        rospy.sleep(1)

        print "stick"
        l_offset = [0, 0.005, -0.005, 0.01, -0.01, 0.015, -0.015, 0.02, -0.02, 0.025, -0.025]
        o = 0
        
        #set a loop to detect a fesible pose to stick
        while o < 11:
            stick_pose = l_arm.get_current_pose(lend_effector_link).pose
            s_wpose = deepcopy(stick_pose)
            s_waypoints = []
            s_waypoints.append(stick_pose)
            s_wpose.position.y += 0.34
            print "1 part"
            rospy.sleep(1)
            s_waypoints.append(deepcopy(s_wpose))
            print "2 part"
            rospy.sleep(1)
            s_fraction = 0.0
            s_maxtries = 100
            s_attempts = 0            
            # Set the internal state to the current state
            #  l_offset = [0.005, 0.01, 0,015, 0.02, 0.025, 0.03, -0.005, -0.01, -0,015, -0.02, -0.025, -0.03]
            
            l_arm.set_start_state_to_current_state()
            while s_fraction < 1.0 and s_attempts < s_maxtries:
                (s_plan, s_fraction) = l_arm.compute_cartesian_path (
                                        s_waypoints,   # waypoint poses
                                        0.01,        # eef_step
                                        0.0,         # jump_threshold
                                        True)        # avoid_collisions
                
            # Increment the number of attempts 
                s_attempts += 1               
            # Print out a progress message
                if attempts % 10 == 0:
                    rospy.loginfo("Still trying after " + str(s_attempts) + " attempts...")
                         
            # If we have a complete plan, execute the trajectory        
            if s_fraction > 0.90:           
                rospy.loginfo(str(s_fraction) + " Moving the arm.")   
                r_arm.execute(s_plan)                          
                rospy.loginfo("Path execution complete.")
                break 

            else:
                s_attempts = 0   
                rospy.loginfo("Path planning failed with only " + str(s_fraction) + " success after " + str(s_maxtries) + " attempts.")  
                l_arm.shift_pose_target(0, -l_offset[o], lend_effector_link)
                o = o + 1
                if o > 11:
                    break
                l_arm.shift_pose_target(0, l_offset[o], lend_effector_link)
                l_arm.go()
                rospy.sleep(2)
   
        #place the object#####################################     
        if not plan.joint_trajectory.points:
            print "[ERROR] No trajectory found"
        else:           
            scene.remove_attached_object(lend_effector_link, 'bolt')

            take_pose = PoseStamped()
            take_pose.header.frame_id = reference_frame
            take_pose.pose.position.x = -0.52
            take_pose.pose.position.y = 0.22
            take_pose.pose.position.z = 0.86
           # take_pose.pose.orientation.x = 0.01
            #take_pose.pose.orientation.y = -3.14
           # take_pose.pose.orientation.z = -3.14
            take_pose.pose.orientation.w = 0           

#            scene.attach_mesh(end_effector_link, 'bolt', take_pose)
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
    

    
