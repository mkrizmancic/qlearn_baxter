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
        T_GB_p = Vector(-0.0924028561603, -0.0172428611168, -0.944187986747)
        T_GB_Q = Rotation.Quaternion(-0.00993130, 0.00080579, -0.00441977, 0.99994059)  # qx,qy,qz,qw
        T_GB = Frame(T_GB_Q, T_GB_p)

        T_empty_p = Vector(0, 0, 0)
        T_empty_Q = Rotation.Quaternion(0, 0, 0, 1)
        T_empty = Frame(T_empty_Q, T_empty_p)

        # Sending new transformations
        br.sendTransform(T_GB.p, T_GB.M.GetQuaternion(), rospy.Time.now(), 'base', 'bax_head')
        br.sendTransform(T_GB.p, T_GB.M.GetQuaternion(), rospy.Time.now(), 'reference/base', 'bax_head')
        br.sendTransform(T_empty.p, T_empty.M.GetQuaternion(), rospy.Time.now(), 'world', 'base')
        rate.sleep()
