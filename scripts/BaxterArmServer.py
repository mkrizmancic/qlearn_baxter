#!/usr/bin/env python
"""
CREDIT:
A part of the script (a few methods) was taken from graduation thesis by Miriam Bilac
at Faculty of Electrical Engineering and Computing, University of Zagreb.
"""

import actionlib
import moveit_commander
import moveit_msgs.msg
from baxter_interface import Gripper, Limb
from baxter_moveit_config.msg import baxterAction, baxterGoal, baxterResult, baxterFeedback
from geometry_msgs.msg import Pose, Point

import ik_solver
from Util import *

VALID_LIMB_JOINTS = 1
INVALID_LIMB_JOINTS = 0


# Comments beginning with "noinspection" are PyCharm auto-generated comments
# noinspection PyMethodMayBeStatic,PyAttributeOutsideInit,PyUnusedLocal

class BaxterArm:
    """Actionlib server for Baxter's arm. See more here: http://wiki.ros.org/actionlib"""
    def __init__(self, arm):
        """ Initialize and start actionlib server. """
        self.as_goal = {'left': baxterGoal(), 'right': baxterGoal()}
        self.as_feed = {'left': baxterFeedback(), 'right': baxterFeedback()}
        self.as_res = {'left': baxterResult(), 'right': baxterResult()}
        self.action_server_left = actionlib.SimpleActionServer("baxter_action_server_left", baxterAction,
                                                               self.execute_left, auto_start=False)
        self.action_server_right = actionlib.SimpleActionServer("baxter_action_server_right", baxterAction,
                                                                self.execute_right, auto_start=False)

        self.left_arm = Limb('left')
        self.right_arm = Limb('right')
        self.left_gripper = Gripper('left')
        self.right_gripper = Gripper('right')
        self.left_gripper.calibrate()

        self.arm = arm

    def set_to_current_position(self, point_target, arm):
        point_target.position.x = arm.endpoint_pose()['position'].x
        point_target.position.y = arm.endpoint_pose()['position'].y
        point_target.position.z = arm.endpoint_pose()['position'].z

        point_target.orientation.x = arm.endpoint_pose()['orientation'].x
        point_target.orientation.y = arm.endpoint_pose()['orientation'].y
        point_target.orientation.z = arm.endpoint_pose()['orientation'].z
        point_target.orientation.w = arm.endpoint_pose()['orientation'].w

    def set_goal(self, point_target, arm):
        point_target.position.x = self.as_goal[arm].pose.position.x
        point_target.position.y = self.as_goal[arm].pose.position.y
        point_target.position.z = self.as_goal[arm].pose.position.z

        point_target.orientation.x = self.as_goal[arm].pose.orientation.x
        point_target.orientation.y = self.as_goal[arm].pose.orientation.y
        point_target.orientation.z = self.as_goal[arm].pose.orientation.z
        point_target.orientation.w = self.as_goal[arm].pose.orientation.w

    def execute(self, goal, arm):
        if arm == 'right':
            chosen_arm = self.right_arm
            chosen_gripper = self.right_gripper
            chosen_server = self.action_server_right
        else:
            chosen_arm = self.left_arm
            chosen_gripper = self.left_gripper
            chosen_server = self.action_server_left
            self.group = moveit_commander.MoveGroupCommander("left_arm")

        self.as_goal[arm].id = goal.id
        self.as_goal[arm].pose.position = goal.pose.position
        self.as_goal[arm].pose.orientation = goal.pose.orientation

        pose_target = Pose()
        self.set_to_current_position(pose_target, chosen_arm)

        display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path',
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=1)

        if goal.id == 1:
            user_print("ODLAZAK NA ZADANU POZICIJU", 'info')
            self.set_goal(pose_target, arm)
            limb_joints = ik_solver.ik_solve(arm, Point(pose_target.position.x, pose_target.position.y,
                                                        pose_target.position.z - 0.03), pose_target.orientation)
                                                        # z is offset because point for calculating IK is
                                                        # a bit different than actual tool endpoint
            if limb_joints != 0:
                chosen_arm.move_to_joint_positions(limb_joints)
                self.as_res[arm].status = VALID_LIMB_JOINTS
                self.as_res[arm].id = self.as_goal[arm].id
                chosen_server.set_succeeded(result=self.as_res[arm])
            else:
                self.as_res[arm].status = 0
                self.as_res[arm].id = 0
                chosen_server.set_aborted(result=self.as_res[arm])

        if goal.id == 2:
            user_print("OTVARANJE GRIPPERA", 'info')
            chosen_gripper.open()
            self.as_res[arm].status = VALID_LIMB_JOINTS
            self.as_res[arm].id = self.as_goal[arm].id
            chosen_server.set_succeeded(result=self.as_res[arm])

        if goal.id == 3:
            user_print("ZATVARANJE GRIPPERA", 'info')
            chosen_gripper.close()
            self.as_res[arm].status = VALID_LIMB_JOINTS
            self.as_res[arm].id = self.as_goal[arm].id
            chosen_server.set_succeeded(result=self.as_res[arm])

    def execute_right(self, goal):
        self.execute(goal, 'right')

    def execute_left(self, goal):
        self.execute(goal, 'left')

    def start(self):
        self.action_server_left.start()
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()


if __name__ == '__main__':
    rospy.init_node('Baxter_Server')
    try:
        baxter_server = BaxterArm('left')
        baxter_server.start()
    except rospy.ROSInterruptException:
        rospy.loginfo('Terminating baxter_action_server.')
