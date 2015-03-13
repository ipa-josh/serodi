#!/usr/bin/env python

import sys
import rospy
import cob_srvs.srv
import sensor_msgs.msg
import time

global sub_js

def recover_base():
    rospy.wait_for_service('/base/recover')
    try:
        recover = rospy.ServiceProxy('/base/recover', cob_srvs.srv.Trigger)
        return recover()
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

needs_recover = False
def cb_em(data):
	global needs_recover
	em = 
	needs_recover = (needs_recover or em)
	if needs_recover and not em:
		recover_base()
		time.sleep(5)
		
def cb_js(data):
	#spawn here
	global sub_js
	sub_js.unregister()

if __name__ == "__main__":
	rospy.init_node('background', anonymous=True)
    rospy.Subscriber("//emergency", String, cb_em, 1)
    sub_js = rospy.Subscriber("/joint_states", sensor_msgs.msg.JointState, cb_js, 1)
    rospy.spin()
