#!/usr/bin/env python 
"""
This module is used for broadcasting transformations from Optitrack
coordinate frame during normal operation.
Do not confuse it with optitrack_find.py which is used while searching for transformations.
"""
import rospy
import tf
from geometry_msgs.msg import PoseStamped


class OptitrackNode:
    """
    Optitrack ROS node for space positioning and broadcasting transformations.

    This node takes position data from Optitrack system and performs axis adjustments.
    Optitrack uses y-up coordinate system while z-up coordinate system is used more often.
    After adjusting, broadcast coordinates as transformations from optitrack's coordinate frame.

    Attributes:
        br_head: Broadcasting transformation for Baxter's head
        br_rods: Broadcasting transformation for rods location
    """

    def __init__(self):
        """Set up class variables, initialize broadcaster and start subscribers."""

        # Create broadcasters
        self.br_head = tf.TransformBroadcaster()
        self.br_rods = tf.TransformBroadcaster()

        # Create subscribers
        rospy.Subscriber("bax_head/pose", PoseStamped, self.head_callback, queue_size=1)
        rospy.Subscriber("rods/pose", PoseStamped, self.rod_callback, queue_size=1)

        # Main while loop.
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            rate.sleep()

    def head_callback(self, data):
        """Callback function that adjust axis data for our Baxter's head and sends transform."""
        x = data.pose.position.x
        y = -data.pose.position.z
        z = data.pose.position.y

        qx = data.pose.orientation.x
        qy = -data.pose.orientation.z
        qz = data.pose.orientation.y
        qw = data.pose.orientation.w

        # deg = euler_from_quaternion((qx, qy, qz, qw))
        # # print ("{:f} | {:f} | {:f}".format(degrees(deg[0]), degrees(deg[1]), degrees(deg[2])))

        self.br_head.sendTransform((x, y, z), (qx, qy, qz, qw), rospy.Time.now(), "/bax_head", "/optitrack")

    def rod_callback(self, data):
        """Callback function that adjust axis data for rods and sends transform."""
        x = data.pose.position.x
        y = -data.pose.position.z
        z = data.pose.position.y

        qx = data.pose.orientation.x
        qy = -data.pose.orientation.z
        qz = data.pose.orientation.y
        qw = data.pose.orientation.w

        self.br_rods.sendTransform((x, y, z), (qx, qy, qz, qw), rospy.Time.now(), "/rods", "/optitrack")


if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('OptitrackTF')

    # Go to class functions that do all the heavy lifting.
    # Do error checking.
    try:
        on = OptitrackNode()
    except rospy.ROSInterruptException:
        pass
