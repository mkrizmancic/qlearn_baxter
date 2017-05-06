#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
CREDIT: Lucija Kopic (Graduation Thesis)
"""

#TODO Glavni docstring

import time
import sys
import math
from threading import Thread

import rospy
import actionlib
import tf
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose
from baxter_moveit_config.msg import baxterAction, baxterGoal, baxterResult, baxterFeedback

import Errors
from Util import *


class BaxterArmClient():
    """
    This class unfortunately contains some "magic numbers".
    Replacing them with constants would make the code more difficult to read.
    In order to understand what are they representing, please read 'Environment setup' section of README
    """
    def __init__ (self):
        self.left_client = actionlib.SimpleActionClient("baxter_action_server_left", baxterAction)
        self.left_client.wait_for_server()
        self.listener = tf.TransformListener()

    def transformations (self):
        """Transform rods' coordinate system to the Baxter's coordinate system."""
        self.listener.waitForTransform("/base", "/stup", rospy.Time(0), rospy.Duration(8.0))
        (trans, rot) = self.listener.lookupTransform('/base', '/stup', rospy.Time(0))
        return trans

    def position (self, target_position, trans, height):
        """
        Set the position of the robot's arm.

        Args:
            target_position (Pose): Wanted coordinates of robot's tool
            trans: Calculated transformation
            height (float): Height offset, depends on the number of disks on the rod

        Returns:
            target_position (Pose): Modified coordinates and orientation of robot's tool
        """
        roll = -math.pi / 2
        pitch = 0
        yaw = -math.pi / 2
        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        target_position.orientation.x = quaternion[0]
        target_position.orientation.y = quaternion[1]
        target_position.orientation.z = quaternion[2]
        target_position.orientation.w = quaternion[3]
        target_position.position.x = trans[0] + 0.08 + -0.18 - 0.05
        target_position.position.y = trans[1] - 0.05
        target_position.position.z = trans[2] + height - 0.015
        return target_position

    def go_to_position (self, task, destination, height, offset_x, offset_y, offset_z):
        """
        Calculate goal position, send that to the robot and wait for response.

        Args:
            task (string): Pick or place action
            destination (int): Destination rod [0, 1, 2]
            height (float): Height of the goal position (based on number of disk on the rod)
            offset_x, offset_y, offset_z
        """
        #TODO trebaju li mi ti offesti u pozivu ili ih mogu nekako drugacije
        goal = Pose()
        trans = self.transformations()
        if task == 'pick':
            height = get_pick_height(height)
        else: 
            height = get_place_height(height)
        goal = self.position(goal, trans, height)

        # Calculate offset from the markers
        if destination == 0: offset_y -= 0.2
        if destination == 1: offset_y += 0
        if destination == 2: offset_y += 0.2

        # WTF?!
        offset_z += 0.05
        offset_z -= 0.02
        offset_y -= 0.05
        if destination == 0: offset_z -= 0.005
        if destination == 1: offset_z -= 0.005      

        # Update goal with calculated offsets
        goal.position.x = goal.position.x + offset_x
        goal.position.y = goal.position.y + offset_y 
        goal.position.z = goal.position.z + offset_z
        goal_final = baxterGoal(id = 1, pose = goal)
        
        self.left_client.send_goal_and_wait(goal_final)
        result = self.left_client.get_result()

        if result.status:
            return 1 
        else:
            return Errors.RaiseGoToFailed(task, destination, height, offset_x, offset_y, offset_z)

    def close_gripper (self):
        """Send the instruction to the robot to close the gripper."""
        goal = Pose()
        goal_final = baxterGoal(id=3, pose = goal)
        status = self.left_client.send_goal_and_wait(goal_final)
        result = self.left_client.wait_for_result()
    
    def open_gripper (self):
        """Send the instruction to the robot to open the gripper."""
        goal = Pose()
        goal_final = baxterGoal(id=2, pose = goal)
        self.left_client.send_goal(goal_final)
        self.left_client.wait_for_result()  
        
    def pick (self, pick_destination, pick_height):
        """
        Execute the pick action.
        
        Args:
            pick_destination (int): Describes the rod from which to pick up the disk [0, 1, 2]
            pick_height (float): Height from which to pick up the disk
        Returns:
            1 if successful, 0 otherwise
        """
        starting_height = get_pick_height(pick_height)
        # Go in front of the selected rod with the neccessary height
        pick1 = self.go_to_position('pick', pick_destination, pick_height, 0, 0, 0)
        if pick1: 
            print 'PICK 1 ok'
            # Move towards the rod
            pick2 = self.go_to_position('pick', pick_destination, pick_height, 0.1, 0, 0) 
            if pick2: 
                print 'PICK 2 ok'
                # Close the gripper
                pick3 = self.close_gripper()
                if pick3:
                    print 'PICK 3 ok' 
                    # Lift vertically above the rod
                    pick4 = self.go_to_position('pick', pick_destination, pick_height, 0.1, 0, 0.28 - starting_height)
                    if pick4:
                        print 'PICK 4 ok' 
                        return 1
        return Errors.RaisePickFailed() 
        
        
    def place (self, place_destination, place_height):
        """
        Execute the place action.
        
        Args:
            place_destination (int): Describes the rod on which to place the disk [0, 1, 2]
            place_height (float): Height to which disk should be placed
        Returns:
            1 if successful, 0 otherwise
        """
        starting_height = get_place_height(place_height)
        # Go directly above the selected rod
        place1 = self.go_to_position('place', place_destination, place_height, 0.1, 0, 0.28 - starting_height)
        if place1:
            print "PLACE 1 OK"
            # Lower vertically to the neccessary height
            place2 = self.go_to_position('place', place_destination, place_height, 0.1, 0, 0)
            if place2: 
                print "PLACE 2 OK"
                # Open the gripper
                place3 = self.open_gripper()
                if place3: 
                    print "PLACE 3 OK"
                    # Lower the arm slightly more to avoid hitting the diske
                    place4 = self.go_to_position('place', place_destination, place_height, 0.1, 0, -0.015) 
                    if place4: 
                        print "PLACE 4 OK"
                        # Move away from the rod
                        place5 = self.go_to_position('place', place_destination, place_height, 0, 0, -0.015)
                        if place5: 
                            print "PLACE 5 OK"
                            return 1
        return Errors.RaisePlaceFailed()
    
    
    def calibration_rod (self):
        """Calibrate rod positions."""
        # Go to 1st, 2nd and 3rd rod
        self.go_to_position('pick', 0, 1, 0.1 + 0.15, 0, 0)
        rospy.sleep(2)
        self.go_to_position('pick', 1, 1, 0.1 + 0.15, 0, 0)
        rospy.sleep(2)
        self.go_to_position('pick', 2, 1, 0.1 + 0.15, 0, 0)
        rospy.sleep(2)

    def calibration_disk (self):
        """Calibrate disk positions."""
        # Go to disks 1, 2 and 3 on the first rod
        self.go_to_position('pick', 0, 1, 0, 0, 0)
        self.go_to_position('pick', 0, 1, 0.1, 0, 0)
        rospy.sleep(10)
        self.go_to_position('pick', 0, 1, 0.1, 0, 0.01)
        self.go_to_position('pick', 0, 1, 0, 0, 0)

        self.go_to_position('pick', 0, 2, 0, 0, 0)
        self.go_to_position('pick', 0, 2, 0.1, 0, 0)
        rospy.sleep(10)
        self.go_to_position('pick', 0, 2, 0.1, 0, 0.01)
        self.go_to_position('pick', 0, 2, 0, 0, 0)

        self.go_to_position('pick', 0, 3, 0, 0, 0)
        self.go_to_position('pick', 0, 3, 0.1, 0, 0)
        rospy.sleep(10)
        self.go_to_position('pick', 0, 3, 0.1, 0, 0.01)
        self.go_to_position('pick', 0, 3, 0, 0, 0)

    def pick_and_place (self, pick_destination, pick_height, place_destination, place_height):
        """
        Execute 'pick and place' action.

        Args:
            pick_destination (int): Describes the rod from which to pick up the disk [0, 1, 2]
            pick_height (float): Height from which to pick up the disk
            place_destination (int): Describes the rod on which to place the disk [0, 1, 2]
            place_height (float): Height to which disk should be placed

        Returns:
            1 if successful, 0 otherwise
        """
        print '------PICK---------------', pick_destination, pick_height
        pick = self.pick(pick_destination, pick_height)
        if pick == 0: 
            return Errors.RaisePickAndPlaceFailed()
        else:
            print '------PLACE--------------',place_destination, place_height
            place = self.place(place_destination, place_height,)
            if place == 0: 
                return Errors.RaisePickAndPlaceFailed()
            else: 
                return 1
        

    def start (self, pick_destination, pick_height, place_destination, place_height):        
        thread = Thread(target = self.pick_and_place, args=(pick_destination, pick_height, place_destination, place_height))
        thread.start()
        thread.join()

    def kalibracija_kolutovi (self):
        thread = Thread(target = self.calibration_disk, args=(1))
        thread.start()
        thread.join()

    def kalibracija_stupovi (self):
        thread = Thread(target = self.calibration_rod, args=(1))
        thread.start()
        thread.join()

if __name__ == '__main__':
    rospy.init_node('baxter_client',anonymous=True, disable_signals = True)
    try:
        client = BaxterArmClient()
        client.start()  
    except rospy.ROSInterruptException:
        rospy.loginfo('Terminating baxter_client.')
