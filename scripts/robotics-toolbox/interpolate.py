#!/usr/bin/env python
"""
This script records the state of /bax_head to /base transformation and calculates average over time.

One should record state of transformation multiple times and for multiple hand positions.
Use implemented test functions to precisely move the arm to desired position.
Total number of recordings must be the power of 2, e.g. 8 different arm positions with 2
recordings for each.

The script outputs values of transformation matrix that need to be copied to transformations.py
"""
from __future__ import division

import rospy
import tf
import math
from Quaternion import quaternion

def isnan(quat):
    """Check if any part of input quaternion is NaN."""
    if math.isnan(quat.s) or math.isnan(quat.v[0,0]) or math.isnan(quat.v[0,1]) or math.isnan(quat.v[0,2]):
        return True
    else:
        return False


class TF_average:
    """Record transformations and find average to minimise errors."""
    def __init__(self):
        rospy.init_node('listen_to_write', anonymous=True)
        listener = tf.TransformListener()
        translation = []
        rotation = []

        while True:
            key = raw_input('Enter "a" for recording and "f" to calculate Matrix: ')
            if key == 'a':
                print 'You pressed key for listening Transformation! \n'
                for i in range(0, 2):
                    (trans, rot) = listener.lookupTransform('/bax_head', '/base', rospy.Time(0))
                    trans = list(trans)
                    rot = list(rot)
                    temp = rot[-1]  # x,y,z,w ---> w,x,y,z
                    rot[-1] = rot[0]
                    rot[0] = temp
                    translation.append(trans)
                    rotation.append(rot)
            elif key == 'f':
                x = 0
                y = 0
                z = 0
                for i in range(0, len(translation)):
                    x += translation[i][0]
                    y += translation[i][1]
                    z += translation[i][2]
                x /= len(translation)
                y /= len(translation)
                z /= len(translation)

                print "X: ", x
                print "Y: ", y
                print "Z: ", z
                print

                q = rotation
                for i in range(0, len(q)):
                    q[i] = quaternion(q[i])

                q_average = q
                while len(q_average) > 1:
                    q = q_average
                    q_average = []
                    for k in range(0, int(len(q) - 1), 2):
                        average = q[k].interp(q[k + 1], 0.5)
                        if isnan(average):
                            average = q[k]
                        q_average.append(average)

                print "qX: ", average.v[0,0]
                print "qY: ", average.v[0,1]
                print "qZ: ", average.v[0,2]
                print "qW: ", average.s
                return

if __name__ == '__main__':
    try:
        TF_average()
    except rospy.ROSInterruptException:
        pass
