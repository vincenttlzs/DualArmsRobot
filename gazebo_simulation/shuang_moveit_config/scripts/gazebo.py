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

        # Set the height of the table off the ground
        
        
     
        
        print "1"

        # Set the target pose for the arm
        joint_position = [-0.1687561467327905, -2.5930083378294206, -0.15629770273311916, -0.051604258049461116, -1.9780572482821324, -8.013808510351158, 8.842669618758237e-05, -2.200003973024546, -1.5706216792847787, 0.000154300758071102, -0.0009756950005002096, 0.005214215064979655]         
        b_arms.set_joint_value_target(joint_position)       
        b_arms.go()
        rospy.sleep(2) 

        # open the gripper
        print "g1"

        l_gripper.set_joint_value_target(GRIPPER_OPEN)
        l_gripper.go()
        rospy.sleep(2)   
   
        print "2"

        joint_position2 = [-0.14939353357741503, -2.4568493182064906, -1.1219610706570764, -0.0520739587447725, -1.1486334344304456, -8.036337763193378, 6.572239555335813e-05, -2.2000407199919536, -1.5705489958716123, 0.00014147565678968022, -0.0009003915477752145, 0.005353152797577643]  
        b_arms.set_joint_value_target(joint_position2)       
        b_arms.go()
        rospy.sleep(2) 

        print "g2"

        l = -0.03
        ldegree = [l]
        l_gripper.set_joint_value_target(ldegree)
        l_gripper.go()
        rospy.sleep(1)

        print "pickup"

        joint_position2 = [-0.2001200656849571, -3.0310726000509405, -0.2992728657388488, -0.0473849556259065, -1.399025411469049, -8.07393812848713, 3.805023683955966e-05, -2.2001211691491758, -1.5705140579402261, 0.00014161721294136953, -0.0008906545641673702, 0.005541227574739516]  
        b_arms.set_joint_value_target(joint_position2)       
        b_arms.go()
        rospy.sleep(2) 


        joint_position2 = [-0.5791281336271208, -2.7450120537011315, -0.6942927888352246, -0.038429445395854245, -1.305982968309797, -8.455042350659026, 3.433241967698564e-05, -2.2000399629897416, -1.5706487238827211, 0.00018734407636955552, -0.0014996435898275706, -0.004703601237183719]  
        b_arms.set_joint_value_target(joint_position2)       
        b_arms.go()
        rospy.sleep(2) 

       
        joint_position2 = [-0.538279740393917, -2.2299493703894306, -1.0425394129512346, -0.03896687909667129, -1.4715241011118954, -8.4080364953766, 9.489327595169073e-05, -2.200005807175309, -1.5707206419308264, 0.00017597983761152847, -0.001180214058597251, -0.005057502675047232]  
        b_arms.set_joint_value_target(joint_position2)       
        b_arms.go()
        rospy.sleep(2) 
   
        print "open"     

        l_gripper.set_joint_value_target(GRIPPER_OPEN)
        l_gripper.go()
        rospy.sleep(2)   
        #catch the object


        
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
    

    
