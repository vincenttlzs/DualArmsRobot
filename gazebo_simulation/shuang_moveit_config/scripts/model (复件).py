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
 plan = r_gripper.plan()

        q = -0.5
             
        
        while q < 0.45:#not rospy.is_shutdown():
            degree = [q]
            r_gripper.set_joint_value_target(degree)

            rospy.loginfo(str(r))
            if not plan.joint_trajectory.points:
                
                q = q + 0.05
                rospy.loginfo("attempt" + str(q))
                continue
   
          #  elif q > 0.3:
           #r_gripper.go()
              #  break
            
            else:
                   #group.go(wait=True)
                 q = q - 0.05
                 rospy.loginfo("final" + str(q))
                 
                
                 break



import rospy, sys
import moveit_commander
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import  PlanningScene, ObjectColor, MoveItErrorCodes, MotionPlanDetailedResponse
from geometry_msgs.msg import PoseStamped, Pose
from scipy.spatial.distance import euclidean 
import shlex, subprocess
import os

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
       
        # Allow replanning to increase the odds of a solution
        r_arm.allow_replanning(True)
        l_arm.allow_replanning(True)
        # Set the reference frame for pose targets
        reference_frame = 'base_footprint'
        
        # Set the right arm reference frame accordingly
        r_arm.set_pose_reference_frame(reference_frame)
        l_arm.set_pose_reference_frame(reference_frame)
        
        # Allow 5 seconds per planning attempt
        r_arm.set_planning_time(5)
        l_arm.set_planning_time(5)
        
        # Give each of the scene objects a unique name
        table_id = 'table'
        box1_id = 'box1'
        sheeve_id = 'sheeve'
        bolt_id = 'bolt'
        
        # Remove leftover objects from a previous run
        scene.remove_attached_object(end_effector_link, 'sheeve')
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
        box1_size = [0.1, 0.05, 0.05]
        
        
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
        box1_pose.pose.position.y = -0.1
        box1_pose.pose.position.z = table_ground + table_size[2] + box1_size[2] / 2.0
        box1_pose.pose.orientation.w = 1.0   
        scene.add_box(box1_id, box1_pose, box1_size)
        
        sheeve_pose = PoseStamped()
        sheeve_pose.header.frame_id = reference_frame
        sheeve_pose.pose.position.x = -0.6310 -0.06
        sheeve_pose.pose.position.y = 0.20 
        sheeve_pose.pose.position.z = 0.7#table_ground + table_size[2] + 0.2

        sheeve_pose.pose.orientation.x = 0
        sheeve_pose.pose.orientation.y = 0
        sheeve_pose.pose.orientation.z = 0 
        sheeve_pose.pose.orientation.w = 0
        scene.add_mesh('sheeve', sheeve_pose, filename = '/home/zs/catkin_ws/src/shuang/meshes/TONG2.STL', size = (0.0008, 0.0008, 0.0008))
        rospy.sleep(1)
        
        bolt_pose = PoseStamped()
        bolt_pose.header.frame_id = reference_frame
        bolt_pose.pose.position.x = -0.6310 -0.06
        bolt_pose.pose.position.y = -0.2
        bolt_pose.pose.position.z = 0.8 #table_ground + table_size[2] + 0.2

        bolt_pose.pose.orientation.x = 1.57
        bolt_pose.pose.orientation.y = 0
        bolt_pose.pose.orientation.z = 0
        bolt_pose.pose.orientation.w = 0
        scene.add_mesh('bolt', bolt_pose, filename = '/home/zs/catkin_ws/src/shuang/meshes/xiao.STL', size = (1, 1, 1))
        rospy.sleep(1)
        

      #  r_arm.set_named_target('straight')
      #  r_arm.go()
      #  rospy.sleep(1)
        
        # Set the target pose in between the boxes and above the table!!!!!!!!!!
        target_pose = PoseStamped()
        target_pose.header.frame_id = reference_frame
        target_pose.pose.position.x = sheeve_pose.pose.position.x + 0.08
        target_pose.pose.position.y = sheeve_pose.pose.position.y
        target_pose.pose.position.z = sheeve_pose.pose.position.z 
        target_pose.pose.orientation.x = 0
        target_pose.pose.orientation.y = 0
        target_pose.pose.orientation.z = 0.707
        target_pose.pose.orientation.w = 0.707

        ltarget_pose = PoseStamped()
        ltarget_pose.header.frame_id = reference_frame
        ltarget_pose.pose.position.x = bolt_pose.pose.position.x + 0.08
        ltarget_pose.pose.position.y = bolt_pose.pose.position.y
        ltarget_pose.pose.position.z = bolt_pose.pose.position.z 
        ltarget_pose.pose.orientation.x = 0
        ltarget_pose.pose.orientation.y = 0
        ltarget_pose.pose.orientation.z = 0.70
        ltarget_pose.pose.orientation.w = 0.707
        
        # Set the target pose for the arm
        r_arm.set_pose_target(target_pose, end_effector_link)
        l_arm.set_pose_target(ltarget_pose, lend_effector_link)

        b_arms.go()   

        while not rospy.is_shutdown():
            #result = self.error_code() #MoveItErrorCodes()
            b_arms.go()
            rospy.sleep(2)
            cu_pose = r_arm.get_current_pose(end_effector_link) 
            lcu_pose = l_arm.get_current_pose(lend_effector_link) 
            # rospy.loginfo(str(MoveItErrorCodes))
            #rospy.loginfo(str(result))
 
            p0 = [sheeve_pose.pose.position.x, sheeve_pose.pose.position.y, sheeve_pose.pose.position.z ]

            p1 = [target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z]
            p2 = [cu_pose.pose.position.x, cu_pose.pose.position.y, cu_pose.pose.position.z]

         

            dist_y = euclidean([0, target_pose.pose.position.y, 0], [0, cu_pose.pose.position.y, 0])
            ldist_y = euclidean([0, ltarget_pose.pose.position.y, 0], [0, lcu_pose.pose.position.y, 0])
            

            dist_target = euclidean(p0, p1)
            rospy.loginfo(str(dist_y))
            rospy.loginfo(str(ldist_y))

            if dist_y > 0.05:# or ldist_y > 0.05:
                continue

            
         
            else:
                break

       
"""
        a1 = [0, -0.01, -0.01, -0.02, -0.02, -0.02, 0.09, 0.01, 0.02, -0.04]
        a2 = [0, -0.01, -0.01, -0.02, -0.02, -0.02, 0.09, 0.01, 0.02, -0.04]
        j = 1
        #count the a array
        c1 = 0
        c2 = 0        

        while j < 2:
            l_arm.shift_pose_target(0, a1[c1], lend_effector_link)    
            l_arm.shift_pose_target(2, a2[c2], lend_effector_link)
            l_arm.shift_pose_target(1, 0.28, lend_effector_link)
            l_arm.plan()
            plan = l_arm.plan()
            rospy.loginfo(str(c1) + str(c2))
            
            if c1 > 9:
                print "failed"
                break

            if not plan.joint_trajectory.points:
                c2 = c2 + 1
                l_arm.shift_pose_target(1, -0.28, lend_effector_link)  
                
                if c2 > 9:
                    
                    c1 = c1 + 1
                    c2 = 0
           

            else:
                l_arm.go()
                rospy.sleep(2)
                j = 3    
        
        rospy.sleep(2)

"""
   
        

          


#############################################
        
        r_gripper.set_joint_value_target(GRIPPER_OPEN)
        r_gripper.go()
        # Pause for a moment...
        rospy.sleep(2)

        # Shift the end-effector to the right 5cm
        r_arm.shift_pose_target(1, 0.05, end_effector_link)
        r_arm.go()
        rospy.sleep(1)
        r_arm.shift_pose_target(0, -0.06, end_effector_link)
        r_arm.go()
        rospy.sleep(2)

        scene.attach_mesh(end_effector_link, 'sheeve',sheeve_pose)
        rospy.sleep(1)

        r_gripper.set_joint_value_target(GRIPPER_CLOSED)
        r_gripper.go()
        # Pause for a moment...
        rospy.sleep(2)
        
        # Return the arm to the "rest" pose stored in the SRDF file
        r_arm.set_named_target('straight')
        r_arm.go()
        
        # Exit MoveIt cleanly
        moveit_commander.roscpp_shutdown()
        
        # Exit the script        
        moveit_commander.os._exit(0)
        
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
        self.relax_all_servos()
        
        os._exit(0) 

if __name__ == "__main__":
    try:
        MoveItDemo()
    except KeyboardInterrupt:
        raise
    

    
