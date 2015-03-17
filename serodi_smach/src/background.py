#!/usr/bin/env python

import sys, os
import rospy
import cob_srvs.srv
import sensor_msgs.msg
import cob_relayboard.msg
import time
import states.movement 

global sub_js

def trigger_srv(srv):
    rospy.wait_for_service(srv)
    try:
        recover = rospy.ServiceProxy(srv, cob_srvs.srv.Trigger)
        return recover()
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        
def recover_base():
	trigger_srv('/base/recover')

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
	
	os.system("roslaunch serodi_smach spawner.launch &")
	
def restart_base():
	global sub_js
	
	print "TODO base shutdown"
	trigger_srv('/base/recover')
	sub_js = rospy.Subscriber("/joint_states", sensor_msgs.msg.JointState, cb_js)
	trigger_srv('/base/init')

if __name__ == "__main__":
	rospy.init_node('background', anonymous=True)
	rospy.Subscriber("/emergency", cob_relayboard.msg.EmergencyStopState, cb_em)
	sub_js = rospy.Subscriber("/joint_states", sensor_msgs.msg.JointState, cb_js)
	chk_loc = states.movement.CheckLocalization()
	rospy.spin()
