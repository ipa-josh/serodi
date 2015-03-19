#!/usr/bin/env python

import roslib; roslib.load_manifest('serodi_smach')
import rospy
import smach
import smach_ros
from simple_script_server import script
import random, math, copy

# Contains following states:
# - MoveOnPath
# - ReadPose
# - 
# - 
# - 
#
#

#g_mode='linear'
g_mode=''


class MoveOnPath(smach.State):
    def __init__(self, sss, path, reverse=False):
        smach.State.__init__(self, outcomes=['success', 'failed'], input_keys=['text','nonblocking'])
        self.sss = sss
        self.path = path
        self.reverse = reverse

    def execute(self, userdata):
		global g_mode
		if self.reverse:
			p = copy.deepcopy(self.path)
			p.reverse()
			for i in xrange(len(p)):
				p[i][2] = p[i][2]+math.pi
				if p[i][2]>2*math.pi: p[i][2]-=2*math.pi
		else:
			p = copy.deepcopy(self.path)
			
		for pose in p:
			try:
				h = self.sss.move('base', pose[0:3], mode=g_mode)
				h.wait()
			except:
				return 'failed'
		return 'success'


class MoveToPose(smach.State):
    def __init__(self, sss, pose):
        smach.State.__init__(self, outcomes=['success', 'failed'], input_keys=['text','nonblocking'])
        self.sss = sss
        self.pose = pose

    def execute(self, userdata):
		global g_mode
		try:
			h = self.sss.move('base', self.pose[0:3], mode=g_mode)
			if (not hasattr(userdata, 'nonblocking') or userdata.nonblocking==False):
				h.wait()
		except:
			return 'failed'
		return 'success'
		

class MoveRel(smach.State):
    def __init__(self, sss, motion):
        smach.State.__init__(self, outcomes=['success', 'failed'], input_keys=['text','nonblocking'])
        self.sss = sss
        self.motion = motion

    def execute(self, userdata):
		try:
			h = self.sss.move_base_rel('base', self.motion)
			#if (not hasattr(userdata, 'nonblocking') or userdata.nonblocking==False):
			#	h.wait()
		except:
			return 'failed'
		return 'success'
		
import sensor_msgs.msg
class ReadDataForRegistration(smach.State):
    def __init__(self, dest_name):
        smach.State.__init__(self, outcomes=['pass'], input_keys=['data'], output_keys=['data'])
        self.dest_name = dest_name.split('.')
        
    def on_data(self, data):
		self.res_angle = data.angle_increment
		self.data = data.ranges
			
    def wait_for_data(self):
		self.data = None
		while self.data==None and not rospy.is_shutdown():
			time.sleep(0.1)
		return self.data

    def execute(self, userdata):
		self.sub = rospy.Subscriber("/scan_unified", sensor_msgs.msg.LaserScan, self.on_data)
		
		s = [userdata.data]
		for dd in self.dest_name[0:len(self.dest_name)-1]:
			s=[s[0][dd]]
		data = self.wait_for_data()
		for d in data:
			s[0][self.dest_name[len(self.dest_name)-1]].append(d)
		
		self.sub.unregister()
		return 'pass'
		
class MoveRel_Registration(smach.State):
    def __init__(self, sss, data, max_angle = 20./180*math.pi, max_trans=0.3):
        smach.State.__init__(self, outcomes=['success', 'failed'], input_keys=['text','nonblocking'])
        self.sss = sss
        self.teachin = data
        self.max_angle = max_angle
        self.max_trans = max_trans
        self.max_matching_dist = max_trans*1.2
        
    def on_data(self, data):
		self.res_angle = data.angle_increment
		self.data = data.ranges
			
    def get_angle(self, data):
		if len(self.teachin)!=len(data):
			smach.logerr("wrong data size")
			return None
						
		max_i = int(round(self.max_angle/self.res_angle+0.9))
		best = []
		for o in range(-max_i,max_i+1):
			energy = 0
			matched = 0
			for i in xrange(len(data)):
				j = (len(data)+i+o)%len(data)
				delta = abs(data[i]-self.teachin[j])
				if data[i]>0 and self.teachin[j]>0 and delta<self.max_matching_dist:
					energy += delta
					matched += 1
			#print o, "matched ",matched
			if matched<0.3*len(data):
				continue
			energy /= matched
			#print "energy ",energy
			
			if len(best)==0 or best[0]>energy:
				best = [energy, o]
				
		if len(best)==0:
			smach.logerr("could not find any match")
			return None
			
		angle = best[1]*self.res_angle
		print "o -> ", o, angle
		
		return angle
			
    def get_trans(self, data):
		if len(self.teachin)!=len(data):
			smach.logerr("wrong data size")
			return None
						
		sdx = 0
		sdy = 0
		matched = 0
		for i in xrange(len(data)):
			delta = data[i]-self.teachin[i]
			if data[i]>0 and self.teachin[i]>0 and abs(delta)<self.max_matching_dist:
				a = i*self.res_angle
				sdx += math.cos(a)*delta
				sdy += math.sin(a)*delta
				matched += 1
		print "matched ",matched
		if matched<0.3*len(data):
			smach.logerr("could not find enough matches")
			return None
			
		print "trans offset ",[sdx/matched, sdy/matched]
		return [sdx/matched, sdy/matched]
			
    def wait_for_data(self):
		self.data = None
		while self.data==None and not rospy.is_shutdown():
			time.sleep(0.1)
		return self.data
		
    def move(self,m):
		print "going to move relative by ",m
		
		h = self.sss.move_base_rel('base', m)
		#h.wait()

    def execute(self, userdata):
		print self.teachin
		self.sub = rospy.Subscriber("/scan_unified", sensor_msgs.msg.LaserScan, self.on_data)
		
		if True:#try:
			
			#recover rotation
			s = 0
			while s<self.max_angle and not rospy.is_shutdown():
				angle = self.get_angle(self.wait_for_data())
				if angle==None: raise
				if abs(angle)<0.03: break
				self.move([0,0,-angle])
				s+=abs(angle)
				
			
			#recover translation
			s = 0
			while s<self.max_trans and not rospy.is_shutdown():
				trans = self.get_trans(self.wait_for_data())
				if trans==None: raise
				
				dist = math.sqrt( trans[0]*trans[0] + trans[1]*trans[1] )
				if dist<0.03: break
				
				self.move([-trans[0],-trans[1],0])
				s += dist
			
		#except:
		#	self.sub.unregister()
		#	return 'failed'
		self.sub.unregister()
		return 'success'


import cob_srvs.srv, tf
class Explore(smach.State):
    def __init__(self, sss):
        smach.State.__init__(self, outcomes=['success', 'failed'])
        self.sss = sss

    def execute(self, userdata):
		try:
			while True:
				angle=0
				while abs(angle)<0.1:
					angle = random.uniform(-1., 1.)
				for li in xrange(3):
					h = self.sss.move_base_rel('base', [0,0,angle])
					#h.wait()
				
				check = rospy.ServiceProxy('/check_closure', cob_srvs.srv.GetPoseStampedTransformed)
				res = check( cob_srvs.srv.GetPoseStampedTransformedRequest() )
				
				if res.success == True:
					return 'success'
				
				x = res.result.pose.position.x
				y = res.result.pose.position.y
				quaternion = (
					res.result.pose.orientation.x,
					res.result.pose.orientation.y,
					res.result.pose.orientation.z,
					res.result.pose.orientation.w)
				angle = tf.transformations.euler_from_quaternion(quaternion)[2]
				h = self.sss.move('base', [x,y,angle],False)
				h.wait(20.)
			
		except:
			return 'failed'
		return 'success'	

class GetLastPose(smach.State):
    def __init__(self, pose, map_link="/map", base_link="/base_link"):
        smach.State.__init__(self, outcomes=['success', 'failed'], input_keys=['text','nonblocking'])
        self.listener = tf.TransformListener()
        self.map_link = map_link
        self.base_link = base_link
        self.pose = pose

    def execute(self, userdata):
		try:
			(trans,rot) = self.listener.lookupTransform(self.map_link, self.base_link, rospy.Time(0))
			self.pose[0] = [trans[0], trans[1], tf.transformations.euler_from_quaternion(rot)[2]]
		except:
			rospy.sleep(1)
			try:
				(trans,rot) = self.listener.lookupTransform(self.map_link, self.base_link, rospy.Time(0))
				self.pose[0] = [trans[0], trans[1], tf.transformations.euler_from_quaternion(rot)[2]]
			except:
				return 'failed'
		return 'success'
        
import numpy, time
import geometry_msgs.msg
from geometry_msgs.msg import PoseWithCovarianceStamped
import std_srvs.srv
class AutoLocalize(smach.State):
    def __init__(self, sss, init_pose=None):
        smach.State.__init__(self, outcomes=['success', 'failed'])
        self.sss = sss
        self.pub_pose = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
        self.init_pose = init_pose
        self.std_dev = None
		
    def cb_posearray(self, msg):
		ar=[]
		for p in msg.poses:
			quaternion = (
					p.orientation.x,
					p.orientation.y,
					p.orientation.z,
					p.orientation.w)
			ar.append([p.position.x,p.position.y,tf.transformations.euler_from_quaternion(quaternion)[2]])
			
		ang_sum=0
		for a in ar: ang_sum += a[2]
		ang_sum /= max(1, len(ar))
		
		for a in ar: 
			a[2] -= ang_sum
			if a[2]>math.pi: a[2] -= math.pi
			if a[2]<-math.pi: a[2] += math.pi
			
		ar = numpy.array(ar)
		d = numpy.std(ar, axis=0)
		self.std_dev = [ math.sqrt(d[0]*d[0]+d[1]*d[1]), d[2] ]

    def send_base_pose(self, x, y, rot, var_trans, var_rot):
		# convert to pose message
		pwcs = PoseWithCovarianceStamped()
		pwcs.header.stamp = rospy.Time.now()
		pwcs.header.frame_id = "/map"

		pwcs.pose.pose.position.x = x
		pwcs.pose.pose.position.y = y
		pwcs.pose.pose.position.z = 0.0
		q = tf.transformations.quaternion_from_euler(0, 0, rot)
		pwcs.pose.pose.orientation.x = q[0]
		pwcs.pose.pose.orientation.y = q[1]
		pwcs.pose.pose.orientation.z = q[2]
		pwcs.pose.pose.orientation.w = q[3]
		
		pwcs.pose.covariance = \
	[var_trans, 0, 0, 0, 0, 0, 
	0, var_trans, 0 ,0 ,0 ,0,
	0, 0, 0, 0, 0 ,0,
	0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, var_rot]

		self.pub_pose.publish(pwcs)
		
    def global_localization(self):
		gl = rospy.ServiceProxy('/global_localization', std_srvs.srv.Empty)
		return gl( std_srvs.srv.EmptyRequest() )
		
    def execute(self, userdata):
		global g_mode
		self.sub_pa = rospy.Subscriber("/particlecloud", geometry_msgs.msg.PoseArray, self.cb_posearray)
		
		#wait for subscriber
		while self.sub_pa.get_num_connections()<1:
			time.sleep(0.1)
		
		try:
			#set pose
			print "init. pose ",self.init_pose
			
			if self.init_pose!=None:
				self.send_base_pose(self.init_pose[0][0], self.init_pose[0][1], self.init_pose[0][2], 0.5, 0.3)
			else:
				self.global_localization()
				
			tries=0
			while self.std_dev==None or self.std_dev[0]>0.25 or self.std_dev[1]>0.15:
				print "deviation is ",self.std_dev
				
				angle=0
				while abs(angle)<0.1:
					angle = random.uniform(-1., 1.)
				for li in xrange(3):
					h = self.sss.move_base_rel('base', [0,0,angle])
					#h.wait()
				
				#get random pose
				for tries in xrange(10):
					check = rospy.ServiceProxy('/get_pose_free', cob_srvs.srv.GetPoseStampedTransformed)
					res = check( cob_srvs.srv.GetPoseStampedTransformedRequest() )
					if res.success == True: break
					time.sleep(10)
				
				if res.success == False:
					tries += 1
					if tries < 5: continue
					smach.logerr("failed to get a free pose")
					return 'failed'
				tries = 0
				
				#move to pose
				x = res.result.pose.position.x
				y = res.result.pose.position.y
				quaternion = (
					res.result.pose.orientation.x,
					res.result.pose.orientation.y,
					res.result.pose.orientation.z,
					res.result.pose.orientation.w)
				angle = tf.transformations.euler_from_quaternion(quaternion)[2]
				h = self.sss.move('base', [x,y,angle], False,mode=g_mode)
				h.wait(20.)
				
			rospy.set_param('/ui/is_localized', True)
			
		except:
			self.sub_pa.unregister()
			return 'failed'
		self.sub_pa.unregister()
		return 'success'

class CheckLocalization:
    def __init__(self):
        self.std_dev = None
	self.sub_pa = rospy.Subscriber("/particlecloud", geometry_msgs.msg.PoseArray, self.cb_posearray)
        self.pub_pose = rospy.Publisher('/ui/localized', PoseWithCovarianceStamped, queue_size=10)
		
    def cb_posearray(self, msg):
		ar=[]
		for p in msg.poses:
			quaternion = (
					p.orientation.x,
					p.orientation.y,
					p.orientation.z,
					p.orientation.w)
			ar.append([p.position.x,p.position.y,tf.transformations.euler_from_quaternion(quaternion)[2]])
			
		ang_sum=0
		for a in ar: ang_sum += a[2]
		ang_sum /= max(1, len(ar))
		
		for a in ar: 
			a[2] -= ang_sum
			if a[2]>math.pi: a[2] -= math.pi
			if a[2]<-math.pi: a[2] += math.pi
		
		ar = numpy.array(ar)
		d = numpy.std(ar, axis=0)
		self.std_dev = [ math.sqrt(d[0]*d[0]+d[1]*d[1]), d[2] ]
		
		# convert to pose message
		var_trans = self.std_dev[0]
		var_rot = self.std_dev[1]
		
		pwcs = PoseWithCovarianceStamped()
		pwcs.header.stamp = rospy.Time.now()
		pwcs.header.frame_id = "/map"

		pwcs.pose.pose.position.x = 0.0 #just empty...
		pwcs.pose.pose.position.y = 0.0
		pwcs.pose.pose.position.z = 0.0
		q = tf.transformations.quaternion_from_euler(0, 0, 0) #just empty...
		pwcs.pose.pose.orientation.x = q[0]
		pwcs.pose.pose.orientation.y = q[1]
		pwcs.pose.pose.orientation.z = q[2]
		pwcs.pose.pose.orientation.w = q[3]
		
		pwcs.pose.covariance = \
	[var_trans, 0, 0, 0, 0, 0, 
	0, var_trans, 0 ,0 ,0 ,0,
	0, 0, 0, 0, 0 ,0,
	0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, var_rot]

		self.pub_pose.publish(pwcs)


class WaitForMoveBase(smach.State):
    def __init__(self, timeout=None):
        smach.State.__init__(self, outcomes=['success', 'failed'], input_keys=['text','nonblocking'])
        self.timeout = timeout

    def execute(self, userdata):
		global g_mode
		
		try:
			rospy.wait_for_service("/global_localization", self.timeout)
		except:
			return 'failed'
		return 'success'

