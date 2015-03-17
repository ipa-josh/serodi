#!/usr/bin/env python

import roslib; roslib.load_manifest('serodi_smach')
import rospy
import smach
import smach_ros
import copy, random

import std_msgs.msg

# Contains following states:
# - WaitForKey
# - Speak
# - SetVariable
# - SaveYaml
# - LoadYaml
# - IterateVar
# - WaitForButtons
# - ShowMenu
# - SetLight

import sensor_msgs.msg
class WaitForKey(smach.State):
    def __init__(self, wait=None):
        smach.State.__init__(self, outcomes=['key_pressed','canceled'], input_keys=[], output_keys=['key'])
        self.wait = wait
        
class WaitForChoice(smach.State):
    def __init__(self, choices, wait=None):
        smach.State.__init__(self, outcomes=choices+(['canceled', 'unknown'] if wait!=None else ['unknown']))
        self.wait = wait
        self.sub = rospy.Subscriber("/ui/choice", std_msgs.msg.String, self.cb_choice)
        
    def cb_choice(self, data):
		self.choice = data.data

    def execute(self, userdata):
        self.choice = None
        
        slept = 0.
        while not rospy.is_shutdown() and (self.wait==None or slept<self.wait):
			found = True
			if self.choice!=None:
				if self.choice in list(self.get_registered_outcomes()):
					return self.choice
				else:
					print "unknown ", self.choice
					return 'unknown'
			time.sleep(0.1)
			slept += 0.1
        
        return 'canceled'
    def cb_joy(self, data):
		self.key = data

    def execute(self, userdata):
        self.key = None
        self.sub_joy = rospy.Subscriber("chatter", sensor_msgs.msg.Joy, self.cb_joy)
        
        rospy.loginfo('Executing state WaitForKey')
        
        slept = 0.
        while self.key==None and (self.wait==None or slept<self.wait):
			rospy.sleep(0.1)
			slept += 0.1
        
        userdata.key = self.key
        if self.key==None:
            return 'canceled'
        else:
            return 'key_pressed'
            
class WaitForButtons(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['key_pressed','canceled'], input_keys=['wait', 'btn_sets'], output_keys=['key'])
        
    def cb_joy(self, data):
		self.key = data

    def execute(self, userdata):
        self.key = None
        self.sub_joy = rospy.Subscriber("chatter", sensor_msgs.msg.Joy, self.cb_joy)
        
        rospy.loginfo('Executing state WaitForButtons')
        
        slept = 0.
        while (not hasattr(userdata, 'wait') or slept<userdata.wait):
			found = True
			if self.key!=None:
				for s in userdata.btn_sets:
					for b in s.btns:
						if self.key.buttons[b]!=1:
							found=False
							userdata.key = s.name
							break
			if found:
				return 'key_pressed'
			rospy.sleep(0.1)
			slept += 0.1
        
        return 'canceled'
            
import time
class WaitForChoice(smach.State):
    def __init__(self, choices, wait=None):
        smach.State.__init__(self, outcomes=choices+(['canceled', 'unknown'] if wait!=None else ['unknown']))
        self.wait = wait
        self.sub = rospy.Subscriber("/ui/choice", std_msgs.msg.String, self.cb_choice)
        
    def cb_choice(self, data):
		self.choice = data.data

    def execute(self, userdata):
        self.choice = None
        
        slept = 0.
        while not rospy.is_shutdown() and (self.wait==None or slept<self.wait):
			found = True
			if self.choice!=None:
				if self.choice in list(self.get_registered_outcomes()):
					return self.choice
				else:
					print "unknown ", self.choice
					return 'unknown'
			time.sleep(0.1)
			slept += 0.1
        
        return 'canceled'
        

import json, copy
class ReadVariableFromChoice(smach.State):
    def __init__(self, keyword, dest_name, wait=None):
        smach.State.__init__(self, outcomes=['success','failed'], input_keys=['data'], output_keys=['data'])
        self.wait = wait
        self.keyword = keyword
        self.dest_name = dest_name.split('.')
        self.sub = rospy.Subscriber("/ui/choice", std_msgs.msg.String, self.cb_choice)
        
    def cb_choice(self, data):
		self.choice = data.data

    def execute(self, userdata):
        self.choice = None
        
        slept = 0.
        while not rospy.is_shutdown() and (self.wait==None or slept<self.wait):
			found = True
			if self.choice!=None:
				key = self.choice.split(":",1)
				if len(key)==2 and key[0] == self.keyword:
					s = [userdata.data]
					for dd in self.dest_name[0:len(self.dest_name)-1]:
						s=[s[0][dd]]
					s[0][self.dest_name[len(self.dest_name)-1]] = json.loads(key[1])
					return 'success'
				else:
					print "unknown ", key
					return 'failed'
			time.sleep(0.1)
			slept += 0.1
        
        return 'failed'
        
class SendChoice(smach.State):
    def __init__(self, choice):
        smach.State.__init__(self, outcomes=['success','failed'])
        self.msg = std_msgs.msg.String(choice)
        self.pub = rospy.Publisher('/ui/choice', std_msgs.msg.String, queue_size=10)

    def execute(self, userdata):
		if self.pub.get_num_connections()<1: time.sleep(1)
		if self.pub.get_num_connections()<1: return 'failed'
		self.pub.publish(self.msg)        
		return 'success'


class Say(smach.State):
    def __init__(self, sss, text, nonblocking=False):
        smach.State.__init__(self, outcomes=['pass'])
        self.sss = sss
        self.text = text
        self.nonblocking = nonblocking

    def execute(self, userdata):
        h = self.sss.say(self.text, False)
        if self.nonblocking==False:
			h.wait()
        return 'pass'

class SetVariable(smach.State):
    def __init__(self, var_name, dest_name):
        smach.State.__init__(self, outcomes=['pass'], input_keys=['data'], output_keys=['data'])
        self.dest_name = dest_name.split('.')
        self.var_name  = var_name.split('.')

    def execute(self, userdata):
		d = [userdata.data]
		for dd in self.var_name:
			d=[d[0][dd]]
		s = [userdata.data]
		for dd in self.dest_name[0:len(self.dest_name)-1]:
			s=[s[0][dd]]
		s[0][self.dest_name[len(self.dest_name)-1]] = copy.deepcopy(d[0])
		return 'pass'
        
import yaml
class SaveYaml(smach.State):
    def __init__(self, obj, filename):
        smach.State.__init__(self, outcomes=['success'], input_keys=['data'])
        self.obj = obj
        self.filename = filename

    def execute(self, userdata):
		f = open(self.filename,'w')
		f.write( yaml.safe_dump(userdata.data[self.obj]) )
		f.close()
		return 'success'
		
class LoadYaml(smach.State):
    def __init__(self, obj, filename, sm=None):
        smach.State.__init__(self, outcomes=['success','failed'], input_keys=['data'], output_keys=['data'])
        self.obj = obj
        self.filename = filename
        
        if sm!=None:
			if self.execute(sm.userdata)!='success': raise Exception("could not load file: "+filename)

    def execute(self, userdata):
		try:
			f = open(self.filename,'r')
			userdata.data[self.obj] =  yaml.load(f.read())
			f.close()
		except:
			return 'failed'
		return 'success'
        
class IterateVar(smach.State):
    def __init__(self, dest_name):
        smach.State.__init__(self, outcomes=['go','done'], input_keys=['data'], output_keys=['data'])
        self.it=None
        self.dest_name = dest_name.split('.')

    def execute(self, userdata):
		try:
			if self.it==None:
				self.it = iter(userdata.data['iteration_var'])
			s = [userdata.data]
			for dd in self.dest_name[0:len(self.dest_name)-1]:
				s=[s[0][dd]]
			s[0][self.dest_name[len(self.dest_name)-1]] = self.it.next()
		except StopIteration:
			self.it = None
			return 'done'
		return 'go'
        
class SelectRandom(smach.State):
    def __init__(self, transitions):
        smach.State.__init__(self, outcomes=['none']+transitions.keys(), input_keys=['data'], output_keys=['data'])

    def execute(self, userdata):
		l = list(self.get_registered_outcomes())
		l.remove('none')
		if len(l)>0:
			return random.choice(l)
		return 'none'
        
class Wait(smach.State):
    def __init__(self, time):
        smach.State.__init__(self, outcomes=['success'])
        self.time = time

    def execute(self, userdata):
		print "sleeping for "+str(self.time)+" secs."
		time.sleep(self.time)
		return 'success'


import serodi_lights.srv
class SetLight(smach.State):
    def __init__(self, sss, color, light=None):
        smach.State.__init__(self, outcomes=['success','failed'], input_keys=[], output_keys=[])
        self.light = light
        self.color = color
        self.sss = sss

    def execute(self, userdata):
		if self.light!=None:
			addr1 = self.light["addr_fs20"]
			addr2 = addr1[:len(addr1)-1]+str(int(addr1[len(addr1)-1])+1)
			
			req = serodi_lights.srv.FS20SwitchRequest()
			req.housecode = self.light["housecode"]
			
			set_light = rospy.ServiceProxy('/fs20switch', serodi_lights.srv.FS20Switch)
			
			req.addr_fs20 = addr1
			req.on = (self.color=="red")
			res = set_light( req )
			if res.success != True:
				smach.logerr(res.error_msg)
				return 'failed'
			
			req.addr_fs20 = addr2
			req.on = (self.color=="green")
			res = set_light( req )
			if res.success != True:
				smach.logerr(res.error_msg)
				return 'failed'
		else:
			print self.color
			self.sss.set_light("light_base", self.color)
			
		return 'success'

class ShowMenu(smach.State):
    def __init__(self, cmd, topic='/ui/menu'):
        smach.State.__init__(self, outcomes=['success'], input_keys=[], output_keys=[])
        self.pub = rospy.Publisher(topic, std_msgs.msg.String, queue_size=10)
        self.msg = std_msgs.msg.String(cmd)

    def execute(self, userdata):
        self.pub.publish(self.msg)
        return 'success'



import sys
import smach.log;
class SMACHCustomLogger:
    def __init__(self, topic='/ui/log'):
        self.pub = rospy.Publisher(topic, std_msgs.msg.String, queue_size=10)
        self.pref = sys.argv[0]
        smach.log.set_loggers(self.loginfo, self.logwarn, self.logdebug, self.logerror)

    def loginfo(self, msg):
        smach.log.loginfo(msg)
        self.pub.publish(std_msgs.msg.String(self.pref+";INFO;"+msg))

    def logwarn(self, msg):
        smach.log.logwarn(msg)
        self.pub.publish(std_msgs.msg.String(self.pref+";WARN;"+msg))

    def logdebug(self, msg):
        smach.log.logdebug(msg)
        self.pub.publish(std_msgs.msg.String(self.pref+";DEBUG;"+msg))

    def logerror(self, msg):
        smach.log.logerr(msg)
        self.pub.publish(std_msgs.msg.String(self.pref+";ERROR;"+msg))
