#!/usr/bin/env python

import SimpleHTTPServer
import SocketServer
import rospy, os

rospy.init_node('webserver')
PORT = rospy.get_param('~port', 9876)
print "serving at port", PORT

os.chdir("../web")
os.system("http-server -p "+str(PORT))
