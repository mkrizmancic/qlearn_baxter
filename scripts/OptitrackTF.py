#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import Pose

class OptitrackTFNode():

    def quat_to_eul (self, data):
        """ 
        This function takes data in quaternion form, exchanges y and z axis and 
        transforms the data to euler form
        """
        quaternion = (
            data.pose.orientation.x,
            data.pose.orientation.z,
            data.pose.orientation.y,
            data.pose.orientation.w)
        return euler_from_quaternion(quaternion)

    def adjust_robot_axis (self,data):
        """ 
        Callback function that adjust axis data for the robot.
        """
        self.crown.x = data.pose.position.x
        self.crown.y = -data.pose.position.z
        self.crown.z = data.pose.position.y

        print  "KRUNA KRUNA   "self.quat_to_eul(data)
        self.crown.orientation = data.pose.orientation

    def adjust_rods_axis (self, data):
        """
        Callback function that adjusts axis data for the rods.
        """
        self.field.x = data.pose.position.x
        self.field.y = -data.pose.position.z
        
        print "STUP STUP" self.quat_to_eul(data)
        self.rods.orientation = data.pose.orientation


    # Must have __init__(self) function for a class
    def __init__(self):
        # Create transform broadcasters
        br_c = tf.TransformBroadcaster()
        br_r = tf.TransformBroadcaster()
 
        # Set the message to publish as command
        self.crown = Pose()    # Position of the robots head markers
        self.rods = Pose()     # Position of playing field
        
        # Create subscribers
        rospy.Subscriber("Crown/pose", PoseStamped, self.adjust_robot_axis, queue_size=1)
        rospy.Subscriber("Rods/pose", PoseStamped, self.adjust_rods_axis, queue_size=1)
        
        # Main while loop.
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            # Publish transforms
            br_c.sendTransform(self.crown.position, 
                               sefl.crown.orientation,
                               rospy.Time.now(),
                               'crown',
                               'world')
            br_r.sendTransform(self.rods.position,
                               self.rods.orientation,
                               rospy.Time.now(),
                               'rods',
                               'world')
            rate.sleep()
if __name__ == '__main__':
    # Initialize the node and name it
    rospy.init_node('Optitrack_Transform')
    
    # Go to class functions that do all the heavy lifting
    # Do error checking
    try:
        on = OptitrackTFNode()
    except rospy.ROSInterruptException:
        pass