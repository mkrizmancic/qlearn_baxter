"""
This module is used for solving inverse kinematics for Baxter Robot.

Functions:
    ik_solve: Actual implementation of IK solving algorithm

CREDIT: Miram Bilac (graduation thesis)
at Faculty of Electrical Engineering and Computing, University of Zagreb
"""

import rospy
from baxter_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Header

import Errors


# Comments beginning with "noinspection" are PyCharm auto-generated comments

def ik_solve(limb, pos, orient):
    ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()

    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    poses = {
        str(limb): PoseStamped(header=hdr, pose=Pose(position=pos, orientation=orient))}

    ikreq.pose_stamp.append(poses[limb])
    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return 1

    if resp.isValid[0]:
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        return limb_joints
    else:
        return Errors.RaiseNotReachable(pos)
