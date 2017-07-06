#!/usr/bin/env python
"""Calculate transformation matrices and broadcast transform from robot's base to head markers."""
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
            (trans_OH, rot_OH) = listener.lookupTransform('/optitrack', '/bax_head', rospy.Time(0))
            (trans_OA, rot_OA) = listener.lookupTransform('/optitrack', '/bax_arm', rospy.Time(0))
            (trans_BG, rot_BG) = listener.lookupTransform('/base', '/left_gripper_base', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        # Rotations
        rot_OH = Rotation.Quaternion(*rot_OH)
        rot_OA = Rotation.Quaternion(*rot_OA)
        rot_BG = Rotation.Quaternion(*rot_BG)
        rot_AG = Rotation.RPY(math.pi / 2, -math.pi, math.pi / 2)

        # Creating Frames
        T_OH = Frame(rot_OH, Vector(*trans_OH))
        T_OA = Frame(rot_OA, Vector(*trans_OA))
        T_BG = Frame(rot_BG, Vector(*trans_BG))
        T_AG = Frame(rot_AG, Vector(0, 0, 0))

        # Finding right transformation
        T_HB = T_OH.Inverse() * T_OA * T_AG * T_BG.Inverse()

        T_empty_p = Vector(0, 0, 0)
        T_empty_Q = Rotation.Quaternion(0, 0, 0, 1)
        T_empty = Frame(T_empty_Q, T_empty_p)

        # Broadcast new transformations
        br.sendTransform(T_HB.p, T_HB.M.GetQuaternion(), rospy.Time.now(), 'base', 'bax_head')
        br.sendTransform(T_HB.p, T_HB.M.GetQuaternion(), rospy.Time.now(), 'reference/base', 'bax_head')
        br.sendTransform(T_empty.p, T_empty.M.GetQuaternion(), rospy.Time.now(), 'world', 'base')
        rate.sleep()
