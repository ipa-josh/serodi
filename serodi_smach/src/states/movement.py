#!/usr/bin/env python

import roslib; roslib.load_manifest('serodi_smach')
import rospy
import smach
import smach_ros
from simple_script_server import script
import random

# Contains following states:
# - MoveOnPath
# - ReadPose
# - 
# - 
# - 
#
#


class MoveOnPath(smach.State):
    def __init__(self, sss, path, reverse=False):
        smach.State.__init__(self, outcomes=['success', 'failed'], input_keys=['text','nonblocking'])
        self.sss = sss
        self.path = path
        self.reverse = reverse

    def execute(self, userdata):
		if reverse:
			p = reversed(self.path)
		else:
			p = self.path
		for pose in p:
			try:
				h = self.sss.move('base', self.pose)
				if (not hasattr(userdata, 'nonblocking') or userdata.nonblocking==False):
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
		try:
			h = self.sss.move('base', self.pose)
			if (not hasattr(userdata, 'nonblocking') or userdata.nonblocking==False):
				h.wait()
		except:
			return 'failed'
		return 'success'


import cob_srvs.srv, tf
class Explore(smach.State):
    def __init__(self, sss):
        smach.State.__init__(self, outcomes=['success', 'failed'])
        self.sss = sss

    def execute(self, userdata):
		if True:
			while True:
				angle = random.uniform(-2.3, 2.3)
				h = self.sss.move('base_rel', [0,0,angle])
				h.wait()
				
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
				h = self.sss.move('base', [x,y,angle])
				h.wait()
			
		#except:
		#	return 'failed'
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
        
import numpy
import geometry_msgs.msg
from geometry_msgs.msg import PoseWithCovarianceStamped
class AutoLocalize(smach.State):
    def __init__(self, sss, init_pose=None):
        smach.State.__init__(self, outcomes=['success', 'failed'])
        self.sss = sss
        self.pub_pose = rospy.Publisher('/initialpose', PoseWithCovarianceStamped)
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
		ar = numpy.array(ar)
		d = numpy.std(ar, axis=0)
		self.std_dev = [ Math.sqrt(d[0]*d[0]+d[1]*d[1]), d[2] ]

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
		self.sub_pa = rospy.Subscriber("/particlecloud", geometry_msgs.msg.PoseArray, self.cb_posearray)
		
		try:
			#set pose
			print "init. pose ",self.init_pose
			
			if self.init_pose!=None:
				self.send_base_pose(self.init_pose[0][0], self.init_pose[0][1], self.init_pose[0][2], 0.5, 0.3)
			else:
				self.global_localization()
				
			while self.std_dev==None or self.std_dev[0]>0.2 or self.std_dev[1]>0.1:
				print "deviation is ",self.std_dev
				
				#get random pose
				check = rospy.ServiceProxy('/get_pose_free', cob_srvs.srv.GetPoseStampedTransformed)
				res = check( cob_srvs.srv.GetPoseStampedTransformedRequest() )
				
				if res.success == False:
					return 'failed'
				
				#move to pose
				x = res.result.pose.position.x
				y = res.result.pose.position.y
				quaternion = (
					res.result.pose.orientation.x,
					res.result.pose.orientation.y,
					res.result.pose.orientation.z,
					res.result.pose.orientation.w)
				angle = tf.transformations.euler_from_quaternion(quaternion)[2]
				h = self.sss.move('base', [x,y,angle])
				h.wait()
			
		except:
			self.sub_pa.unregister()
			return 'failed'
		self.sub_pa.unregister()
		return 'success'
