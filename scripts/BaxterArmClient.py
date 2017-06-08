#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
CREDIT:
Main layout of this file was done by Lucija Kopic (graduation thesis)
at Faculty of Electrical Engineering and Computing, University of Zagreb.
"""

import math
from threading import Thread

import actionlib
import tf
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose
from baxter_moveit_config.msg import baxterAction, baxterGoal
from baxter_interface import Limb

import Errors
from Util import *


# Comments beginning with "noinspection" are PyCharm auto-generated comments
# noinspection PyMethodMayBeStatic,PyUnusedLocal,PyNoneFunctionAssignment,PyRedundantParentheses,PyTypeChecker

class BaxterArmClient:
    """
    Actionlib client for Baxter's arm. See more here: http://wiki.ros.org/actionlib

    This class unfortunately contains some "magic numbers".
    Replacing them with constants would make the code more difficult to read.
    In order to understand what are they representing, 
    please read 'Environment setup' section of README
    """

    def __init__(self):
        """ Initialize and start actionlib client. """
        self.left_client = actionlib.SimpleActionClient("baxter_action_server_left", baxterAction)
        self.left_client.wait_for_server(rospy.Duration(10.0))
        self.listener = tf.TransformListener()
        self.left_arm = Limb('left')

        # Get ROS parameters set up from launch file
        self.left_rod_offset = rospy.get_param('~left_rod')
        self.right_rod_offset = rospy.get_param('~right_rod')
        self.center_rod_offset = rospy.get_param('~center_rod')

    def transformations(self):
        """Transform rods' coordinate system to the Baxter's coordinate system."""
        self.listener.waitForTransform("/base", "/rods", rospy.Time(0), rospy.Duration(8.0))
        (trans, rot) = self.listener.lookupTransform('/base', '/rods', rospy.Time(0))
        return trans

    def position(self, target_position, trans, height):
        """
        Calculate simple position of the robot's arm.

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
        target_position.position.x = trans[0]
        target_position.position.y = trans[1]
        target_position.position.z = trans[2] + height
        return target_position

    def go_to_position(self, task, destination, height, offset_x, offset_y, offset_z):
        """
        Calculate goal position, send that to the robot and wait for response.

        Args:
            task (string): Pick or place action
            destination (int): Destination rod [0, 1, 2]
            height (float): Height of the goal position (based on number of disk on the rod)
            offset_x (float): Offset in robot's x axis
            offset_y (float): Offset in robot's y axis
            offset_z (float): Offset in robot's z axis
        """
        goal = Pose()
        trans = self.transformations()
        if task == 'pick':
            height = get_pick_height(height)
        else:
            height = get_place_height(height)
        goal = self.position(goal, trans, height)

        # Calculate offset from the markers
        if destination == 0: offset_y += self.left_rod_offset
        if destination == 1: offset_y += 0
        if destination == 2: offset_y -= self.right_rod_offset
        offset_x -= self.center_rod_offset

        offset_x -= 0.1  # Moving from rod to rod should be done 10 cm in front of them
        offset_x -= 0.04  # Back up a little to compensate for width of the disks

        # Update goal with calculated offsets
        goal.position.x += offset_x
        goal.position.y += offset_y
        goal.position.z += offset_z
        goal_final = baxterGoal(id=1, pose=goal)

        # Send goal to Baxter arm server and wait for result
        self.left_client.send_goal_and_wait(goal_final)
        result = self.left_client.get_result()
        if result.status:
            return 1
        else:
            return Errors.RaiseGoToFailed(task, destination, height, offset_x, offset_y, offset_z)

    def close_gripper(self):
        """Send the instruction to the robot to close the gripper."""
        goal = Pose()
        goal_final = baxterGoal(id=3, pose=goal)
        status = self.left_client.send_goal_and_wait(goal_final)
        result = self.left_client.wait_for_result()
        return result

    def open_gripper(self):
        """Send the instruction to the robot to open the gripper."""
        goal = Pose()
        goal_final = baxterGoal(id=2, pose=goal)
        self.left_client.send_goal_and_wait(goal_final)
        result = self.left_client.wait_for_result()
        return result

    def pick(self, pick_destination, pick_height):
        """
        Execute the pick action.

        Args:
            pick_destination (int): Describes the rod from which to pick up the disk [0, 1, 2]
            pick_height (float): Height from which to pick up the disk
        Returns:
            1 if successful, 0 otherwise
        """
        self.left_arm.set_joint_position_speed(0.4)  # Set higher speed for non-delicate movements
        starting_height = get_pick_height(pick_height)
        # Go in front of the selected rod with the necessary height
        pick1 = self.go_to_position('pick', pick_destination, pick_height, 0, 0, 0)
        if pick1:
            user_print("PICK 1 ok", 'info')
            # Move towards the rod
            pick2 = self.go_to_position('pick', pick_destination, pick_height, 0.1, 0, 0)
            if pick2:
                user_print("PICK 2 ok", 'info')
                # Close the gripper
                pick3 = self.close_gripper()
                if pick3:
                    user_print("PICK 3 ok", 'info')
                    # Lift vertically above the rod - 0.30 is height just above the rods
                    if pick_height < 3:
                        self.left_arm.set_joint_position_speed(0.1)  # Set lower speed for delicate movements
                    pick4 = self.go_to_position('pick', pick_destination, pick_height, 0.1, 0, 0.30 - starting_height)
                    if pick4:
                        user_print("PICK 4 ok", 'info')
                        return 1
        return Errors.RaisePickFailed()

    def place(self, place_destination, place_height):
        """
        Execute the place action.

        Args:
            place_destination (int): Describes the rod on which to place the disk [0, 1, 2]
            place_height (float): Height to which disk should be placed
        Returns:
            1 if successful, 0 otherwise
        """
        starting_height = get_place_height(place_height)
        # Go directly above the selected rod - 0.30 is height just above the rods
        self.left_arm.set_joint_position_speed(0.4)  # Set higher speed for non-delicate movements
        place1 = self.go_to_position('place', place_destination, place_height, 0.09, 0, 0.30 - starting_height)
        if place1:
            if place_height < 3:
                self.left_arm.set_joint_position_speed(0.08)  # Set lower speed for delicate movements
            user_print("PLACE 1 OK", 'info')
            # Lower vertically to the necessary height
            place2 = self.go_to_position('place', place_destination, place_height, 0.1, 0, 0)
            self.left_arm.set_joint_position_speed(0.4)  # Set higher speed for non-delicate movements
            if place2:
                user_print("PLACE 2 OK", 'info')
                # Open the gripper
                place3 = self.open_gripper()
                if place3:
                    user_print("PLACE 3 OK", 'info')
                    # Lower the arm slightly more to avoid hitting the disk
                    place4 = self.go_to_position('place', place_destination, place_height, 0.1, 0, -0.015)
                    if place4:
                        user_print("PLACE 4 OK", 'info')
                        # Move away from the rod
                        place5 = self.go_to_position('place', place_destination, place_height, 0, 0, -0.015)
                        if place5:
                            user_print("PLACE 5 OK", 'info')
                            return 1
        return Errors.RaisePlaceFailed()

    def pick_and_place(self, pick_destination, pick_height, place_destination, place_height):
        """
        Execute 'pick and place' action.

        Args:
            pick_destination (int): Describes the rod from which to pick up the disk [0, 1, 2]
            pick_height (int): Height from which to pick up the disk (in number of disks)
            place_destination (int): Describes the rod on which to place the disk [0, 1, 2]
            place_height (int): Height to which disk should be placed (in number of disks)

        Returns:
            1 if successful, 0 otherwise
        """
        user_print("------PICK---------------", 'info')
        print pick_destination, pick_height
        pick = self.pick(pick_destination, pick_height)
        if pick == 0:
            return Errors.RaisePickAndPlaceFailed()
        else:
            user_print("------PLACE--------------", 'info')
            print place_destination, place_height
            place = self.place(place_destination, place_height, )
            if place == 0:
                return Errors.RaisePickAndPlaceFailed()
            else:
                return 1

    def test_absolute(self):
        """ Test robot's ability to position its gripper in absolute coordinates (base frame). """
        goal = Pose()
        roll = -math.pi / 2
        pitch = 0
        yaw = -math.pi / 2
        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        goal.orientation.x = quaternion[0]
        goal.orientation.y = quaternion[1]
        goal.orientation.z = quaternion[2]
        goal.orientation.w = quaternion[3]
        while True:
            end = user_input("Zelite li nastaviti? d/n")
            if end != 'd':
                break
            goal.position.x = float(user_input("X?"))
            goal.position.y = float(user_input("Y?"))
            goal.position.z = float(user_input("Z?"))
            goal_final = baxterGoal(id=1, pose=goal)
            self.left_client.send_goal_and_wait(goal_final)
            result = self.left_client.get_result()

    def test_relative(self):
        """ Test robot's ability to position its gripper relative to a given marker. """
        goal = Pose()
        roll = -math.pi / 2
        pitch = 0
        yaw = -math.pi / 2
        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        goal.orientation.x = quaternion[0]
        goal.orientation.y = quaternion[1]
        goal.orientation.z = quaternion[2]
        goal.orientation.w = quaternion[3]
        while True:
            end = user_input("Zelite li nastaviti? d/n")
            if end != 'd':
                break
            trans = self.transformations()
            goal.position.x = trans[0]
            goal.position.y = trans[1]
            goal.position.z = trans[2]
            offset_x = float(user_input("X?"))
            offset_y = float(user_input("Y?"))
            offset_z = float(user_input("Z?"))
            # Uncomment for testing movement speed as well
            # brzina = float(user_input("Brzina?"))
            # self.left_arm.set_joint_position_speed(brzina)
            goal.position.x += offset_x
            goal.position.y += offset_y
            goal.position.z += offset_z
            goal_final = baxterGoal(id=1, pose=goal)
            self.left_client.send_goal_and_wait(goal_final)
            result = self.left_client.get_result()

    def calibration_single(self, rod):
        """Calibrate disk positions."""
        # Go to 1st disk
        self.go_to_position('pick', rod, 3, 0, 0, 0)
        self.go_to_position('pick', rod, 3, 0.1, 0, 0)
        rospy.sleep(2)
        self.go_to_position('pick', rod, 3, 0, 0, 0)
        # Go to 2nd disk
        self.go_to_position('pick', rod, 2, 0, 0, 0)
        self.go_to_position('pick', rod, 2, 0.1, 0, 0)
        rospy.sleep(2)
        self.go_to_position('pick', rod, 2, 0, 0, 0)
        # Go to 3rd disk
        self.go_to_position('pick', rod, 1, 0, 0, 0)
        self.go_to_position('pick', rod, 1, 0.1, 0, 0)
        rospy.sleep(2)
        self.go_to_position('pick', rod, 1, 0, 0, 0)

    def calibration_all(self):
        """Calibrate each rod."""
        # Go to 1st, 2nd and 3rd rod and calibrate
        self.calibration_single(0)
        self.calibration_single(1)
        self.calibration_single(2)

    def start(self, pick_destination, pick_height, place_destination, place_height):
        thread = Thread(target=self.pick_and_place,
                        args=(pick_destination, pick_height, place_destination, place_height))
        thread.start()
        thread.join()

    def calibration(self):
        thread = Thread(target=self.calibration_all)
        thread.start()
        thread.join()


if __name__ == '__main__':
    rospy.init_node('Baxter_Client', disable_signals=True)
    try:
        client = BaxterArmClient()
        client.start()
    except rospy.ROSInterruptException:
        rospy.loginfo('Terminating baxter_client.')
