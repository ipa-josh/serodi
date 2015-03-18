#!/usr/bin/env python

import roslib; roslib.load_manifest('serodi_smach')
import rospy
import smach
import smach_ros

# Contains following states:
# - InitComponents
# - ROSLaunch
# - ROSKill
# - 
# - 
#
#
#

class InitComponents(smach.State):
    def __init__(self, components, sss):
        smach.State.__init__(self, outcomes=['succeeded','failed'], input_keys=['nonblocking'])
        self.sss = sss
        self.components = components
        
    def block(self, h, userdata):
        if (not hasattr(userdata, 'nonblocking') or userdata.nonblocking==False):
			h.wait()
			return h.get_state()==3
        self.handles.add(h)
        return True

    def execute(self, userdata):
		self.handles = []
		for c in self.components:
			if not self.block(self.sss.init(c,False), userdata):
				return 'failed'
				
		#wait for all componentes to finish
		for h in self.handles:
			h.wait()
			if h.get_state()!=3:
				return 'failed'
				
		return 'succeeded'

import subprocess
class ROSLaunch(smach.State):
    def __init__(self, package, launchfile):
        smach.State.__init__(self, outcomes=['success','failed'], input_keys=['running_processes'], output_keys=['running_processes'])
        self.package = package
        self.launchfile = launchfile

    def execute(self, userdata):
		if not hasattr(userdata,'running_processes'):
			userdata.running_processes={}
		if self.package+' '+self.launchfile in userdata.running_processes:
			userdata.running_processes[self.package+' '+self.launchfile].terminate()
		userdata.running_processes[self.package+' '+self.launchfile] = subprocess.Popen(['roslaunch',self.package,self.launchfile])
		
		rospy.sleep(0.2)
		#Todo: check if true or false
		if userdata.running_processes[self.package+' '+self.launchfile].poll()!=None:
			return 'failed'
				
		return 'success'
        
class ROSKill(smach.State):
    def __init__(self, package, launchfile):
        smach.State.__init__(self, outcomes=['pass'], input_keys=['running_processes'], output_keys=['running_processes'])
        self.package = package
        self.launchfile = launchfile

    def execute(self, userdata):
		if self.package==None:
			found = True
			while found:
				found = False
				for k in userdata.running_processes:
					if k in self.launchfile: continue
					
					userdata.running_processes[k].terminate()
					userdata.running_processes.pop(k)
					found = True
					break
		elif hasattr(userdata,'running_processes') and self.package+' '+self.launchfile in userdata.running_processes:
			userdata.running_processes[self.package+' '+self.launchfile].terminate()
			userdata.running_processes.pop(self.package+' '+self.launchfile)
				
		return 'pass'
		
import os
class System(smach.State):
    def __init__(self, cmd):
        smach.State.__init__(self, outcomes=['success','failed'])
        self.cmd = cmd

    def execute(self, userdata):
		if not os.system(self.cmd):
			return 'failed'
				
		return 'success'
