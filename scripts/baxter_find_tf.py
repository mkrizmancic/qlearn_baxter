#!/usr/bin/env python  
import rospy
import tf
import math
from PyKDL import Vector, Frame, Rotation

if __name__ == '__main__':
    rospy.init_node('baxter_find_transformation')

    listener = tf.TransformListener()
    br = tf.TransformBroadcaster()

    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        try:
            (trans_OG, rot_OG) = listener.lookupTransform('/optitrack', '/bax_head', rospy.Time(0))
            (trans_OA, rot_OA) = listener.lookupTransform('/optitrack', '/bax_arm', rospy.Time(0))
            (trans_BC, rot_BC) = listener.lookupTransform('/base', '/left_gripper_base', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        # Rotations->setting all frames in same direction !! ------------------------
        rot_OG = Rotation.Quaternion(*rot_OG)
        rot_OA = Rotation.Quaternion(*rot_OA)
        rot_BC = Rotation.Quaternion(*rot_BC)
        rot_AC = Rotation.RPY(math.pi / 2, -math.pi, math.pi / 2)

        # creating Frames -----------------------------------------------------------
        T_OG = Frame(rot_OG, Vector(*trans_OG))
        T_OA = Frame(rot_OA, Vector(*trans_OA))
        T_BC = Frame(rot_BC, Vector(*trans_BC))
        T_AC = Frame(rot_AC, Vector(0, 0, 0))

        # finding right Transformation ----------------------------------------------
        T_GB = T_OG.Inverse() * T_OA * T_AC * T_BC.Inverse()

        T_empty_p = Vector(0, 0, 0)
        T_empty_Q = Rotation.Quaternion(0, 0, 0, 1)
        T_empty = Frame(T_empty_Q, T_empty_p)

        # sending new Transformations -----------------------------------------------
        br.sendTransform(T_GB.p, T_GB.M.GetQuaternion(), rospy.Time.now(), 'base', 'bax_head')
        br.sendTransform(T_GB.p, T_GB.M.GetQuaternion(), rospy.Time.now(), 'reference/base', 'bax_head')
        br.sendTransform(T_empty.p, T_empty.M.GetQuaternion(), rospy.Time.now(), 'world', 'base')
        rate.sleep()
