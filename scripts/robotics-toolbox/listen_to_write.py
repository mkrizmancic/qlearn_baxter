#!/usr/bin/env python
from __future__ import division

import rospy
import tf
from Quaternion import quaternion


class WriteInFile:
    def __init__(self):
        rospy.init_node('listen_to_write', anonymous=True)
        listener = tf.TransformListener()
        translation = []
        rotation = []
        while True:
            key = raw_input('Enter "a" for recording and "f" for calculate Matrix: ')
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

                print x
                print y
                print z

                q = rotation
                for i in range(0, len(q)):
                    q[i] = quaternion(q[i])

                q_average = []
                for k in range(0, int(len(q) - 1)):
                    if k % 2 == 0:
                        q_average.append(q[k].interp(q[k + 1], 0.5))
                print q_average

                yes_no = raw_input('Do you want to continue? yes or no: ')
                if yes_no == 'yes':
                    q = q_average
                    q_average = []
                    for k in range(0, len(q) - 1):
                        q_average.append(q[k].interp(q[k + 1], 0.5))
                    print q_average
                    yes_no_2 = raw_input('Do you want to continue? yes or no: ')
                    if yes_no_2 == 'yes':
                        q = q_average
                        q_average = []
                        for k in range(0, len(q) - 1):
                            q_average.append(q[k].interp(q[k + 1], 0.5))
                        print q_average
                        return
                    elif yes_no_2 == 'no':
                        return
                elif yes_no == 'no':
                    return


if __name__ == '__main__':
    try:
        WriteInFile()
    except rospy.ROSInterruptException:
        pass
