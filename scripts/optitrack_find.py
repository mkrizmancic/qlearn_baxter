#!/usr/bin/env python
"""
This module is used for broadcasting transformations from Optitrack
coordinate frame while looking for transformations to Baxter's frame.
Do not confuse it with OptitrackTF.py which is used during normal operation.
"""
import rospy
import tf
from geometry_msgs.msg import PoseStamped


class OptitrackFind:
    """
    Optitrack ROS node for space positioning and broadcasting transformations.

    This node takes position data from Optitrack system and performs axis adjustments.
    Optitrack uses y-up coordinate system while z-up coordinate system is used more often.
    After adjusting, broadcast coordinates as transformations from optitrack's coordinate frame.

    Attributes:
        br_head: Broadcasting transformation for Baxter's head
        br_arm: Broadcasting transformation for Baxter's arm
    """

    def __init__(self):
        """ Set up class variables, initialize broadcaster and start subscribers. """

        # Create broadcasters
        self.br_head = tf.TransformBroadcaster()
        self.br_arm = tf.TransformBroadcaster()

        # Create subscribers
        rospy.Subscriber("bax_head/pose", PoseStamped, self.head_callback, queue_size=1)
        rospy.Subscriber("bax_arm/pose", PoseStamped, self.arm_callback, queue_size=1)

        # Main while loop.
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            rate.sleep()

    def head_callback(self, data):
        """ Callback function that adjust axis data for our Baxter's head and sends transform. """
        x = data.pose.position.x
        y = -data.pose.position.z
        z = data.pose.position.y

        qx = data.pose.orientation.x
        qy = -data.pose.orientation.z
        qz = data.pose.orientation.y
        qw = data.pose.orientation.w

        self.br_head.sendTransform((x, y, z), (qx, qy, qz, qw), rospy.Time.now(), "/bax_head", "/optitrack")

    def arm_callback(self, data):
        """ Callback function that adjust axis data for our Baxter's arm and sends transform. """
        x = data.pose.position.x
        y = -data.pose.position.z
        z = data.pose.position.y

        qx = data.pose.orientation.x
        qy = -data.pose.orientation.z
        qz = data.pose.orientation.y
        qw = data.pose.orientation.w

        self.br_arm.sendTransform((x, y, z), (qx, qy, qz, qw), rospy.Time.now(), "/bax_arm", "/optitrack")


if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('Optitrack_find')

    # Go to class functions that do all the heavy lifting.
    # Do error checking.
    try:
        of = OptitrackFind()
    except rospy.ROSInterruptException:
        pass
