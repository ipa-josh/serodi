#!/usr/bin/python
import roslib; roslib.load_manifest('serodi_app')
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped
import socket, math, os, time
import tf

class AndroidConnection:
	def __init__(self, port=38300):
		os.system("cd /home/mobina/adb/adb_git; adb start-server")
		os.system("adb wait-for-device")
		os.system("adb forward tcp:"+str(port)+" tcp:"+str(port))
		self.port = port
		self.connected = False
		self.last_recv = 0

	def connect(self):
		print "connecting..."
		os.system("adb shell am force-stop com.example.extendeddevice")
		os.system("adb shell am start -n com.example.extendeddevice/de.fraunhofer.ipa.Main")

		self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
		self.s.settimeout(10)
		try:
			self.s.connect(("127.0.0.1", self.port))
		except:
			self.close()
			return
		#self.s.setblocking(0)
		self.last_recv = time.clock()
		self.connected = True
		print "connected"

	def close(self):
		self.s.close()
		self.connected = False

	def read(self):
		if not self.connected: self.connect()
		try:
			r = self.s.recv(1024)
			#print len(r), r
			if len(r)>0: self.last_recv = time.clock()
			elif (time.clock()-self.last_recv)>10:
				self.close()
			return r
		except:
			self.close()
			print "disconnected"
			return ""

	def write(self, msg):
		if not self.connected: self.connect()
		try:
			self.s.send(msg)
		except socket.timeout:
			self.close()

con = AndroidConnection()

def handle_calls(req):
	global con
	con.write(req.str)
	return StringSrvResponse()

def talker():
    global con
    pub_va = rospy.Publisher('imu/data_raw', Imu)
    pub_m = rospy.Publisher('imu/mag', Vector3Stamped)
    srv = rospy.Service('/tablet/command', StringSrv, handle_calls)
    rospy.init_node('imu')

    imu = Imu()
    mag = Vector3Stamped()
    last = ""

    imu.header.frame_id = mag.header.frame_id = "/tablet"
    v = math.pow(0.000028,2)
    imu.angular_velocity_covariance    = [v,0,0,  0,v,0, 0,0,v]
    imu.linear_acceleration_covariance = [v,0,0,  0,v,0, 0,0,v]
    imu.orientation_covariance = [v,0,0,  0,v,0, 0,0,v]

    while not rospy.is_shutdown():
	s = last + con.read()
	last = ""
	up_imu=False
	up_mag=False
	for l in s.split("\n"):

		if l[-1:]!='|':
			last = l
			continue
		s = l[:-1].split(", ")
		try:
			for i in range(0, (len(s)-1)/4):
				t = int(s[i*4+1])
				x = float(s[i*4+2])
				y = float(s[i*4+3])
				z = float(s[i*4+4])
				if t==3:
					imu.linear_acceleration.x = x
					imu.linear_acceleration.y = y
					imu.linear_acceleration.z = z
					up_imu=True
				elif t==4:
					imu.angular_velocity.x = x
					imu.angular_velocity.y = y
					imu.angular_velocity.z = z
					up_imu=True
				elif t==5:
					l = math.sqrt(x*x+y*y+z*z)
					mag.vector.x = x/l
					mag.vector.y = y/l
					mag.vector.z = z/l
					up_mag=True
				elif t==6:
					qor =  tf.transformations.quaternion_from_euler(x,y,z)
					imu.orientation.x = qor[0]
					imu.orientation.y = qor[1]
					imu.orientation.z = qor[2]
					imu.orientation.w = qor[3]
		except Exception as e:
			print e

	if up_imu:
		imu.header.stamp = rospy.Time.now()
		pub_va.publish(imu)
	if up_mag:
		mag.header.stamp = rospy.Time.now()
		pub_m.publish(mag)
    con.close()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
