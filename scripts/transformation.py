#!/usr/bin/env python
"""
This script broadcasts transformations from Optitrack
to Baxter coordinate frame. Values for T_GB are constant.
How to get those numbers is explained in README.md
"""
import rospy
import tf
from PyKDL import Vector, Frame, Rotation

if __name__ == '__main__':
    rospy.init_node('BaxterTF')

    listener = tf.TransformListener()
    br = tf.TransformBroadcaster()

    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        T_GB_p = Vector(-0.0833537421765, -0.00865195388513, -0.903851375509)
        T_GB_Q = Rotation.Quaternion(0.005538, 0.003778, -0.001947, 0.999976)  # qx,qy,qz,qw
        T_GB = Frame(T_GB_Q, T_GB_p)

        T_empty_p = Vector(0, 0, 0)
        T_empty_Q = Rotation.Quaternion(0, 0, 0, 1)
        T_empty = Frame(T_empty_Q, T_empty_p)

        # Sending new transformations
        br.sendTransform(T_GB.p, T_GB.M.GetQuaternion(), rospy.Time.now(), 'base', 'bax_head')
        br.sendTransform(T_GB.p, T_GB.M.GetQuaternion(), rospy.Time.now(), 'reference/base', 'bax_head')
        br.sendTransform(T_empty.p, T_empty.M.GetQuaternion(), rospy.Time.now(), 'world', 'base')
        rate.sleep()
