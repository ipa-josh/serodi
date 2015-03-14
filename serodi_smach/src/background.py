#!/usr/bin/env python

import sys, os
import rospy
import cob_srvs.srv
import sensor_msgs.msg
import cob_relayboard.msg
import time
import states.movement 

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
	em = (data.emergency_state==1)
	needs_recover = (needs_recover or em)
	if needs_recover and not em:
		recover_base()
		time.sleep(5)
		
def cb_js(data):
	#spawn here
	global sub_js
	sub_js.unregister()
	
	os.system("roslaunch serodi_mapping spawner.launch &")

if __name__ == "__main__":
	rospy.init_node('background', anonymous=True)
	rospy.Subscriber("/emergency", cob_relayboard.msg.EmergencyStopState, cb_em, 1)
	sub_js = rospy.Subscriber("/joint_states", sensor_msgs.msg.JointState, cb_js, 1)
	chk_loc = states.movement.CheckLocalization()
	rospy.spin()
