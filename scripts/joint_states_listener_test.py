#!/usr/bin/env python
#test client for joint_states_listener

import roslib
roslib.load_manifest('sherpa_spatial_relations')
import rospy
from sherpa_spatial_relations.srv import ReturnJointStates
import time
import sys

def call_return_joint_states(joint_names):
    rospy.wait_for_service("return_joint_states")
    try:
        s = rospy.ServiceProxy("return_joint_states", ReturnJointStates)
        resp = s(joint_names)
    except rospy.ServiceException, e:
        print "error when calling return_joint_states: %s"%e
        sys.exit(1)
    for (ind, joint_name) in enumerate(joint_names):
        if(not resp.found[ind]):
            print "joint %s not found!"%joint_name
    return (resp.position, resp.velocity, resp.effort)


#pretty-print list to string
def pplist(list):
    return ' '.join(['%2.3f'%x for x in list])


#print out the positions, velocities, and efforts of the right shoulder joints
if __name__ == "__main__":
    joint_names = ["right_shoulder_joint_x",
                   "right_shoulder_joint_y",
                    "right_shoulder_joint_z"
                 ]

    while(1):
        (position, velocity, effort) = call_return_joint_states(joint_names)
        print "position:", pplist(position)
        print "velocity:", pplist(velocity)
        print "effort:", pplist(effort)
        time.sleep(1)
