#!/usr/bin/env python

import rospy
from serodi_lights.srv import *
import time

def handle_switch(req):
    r = FS20SwitchResponse()
    r.success = True
    r.error_msg = "just a dummy"
    return r

if __name__ == "__main__":
    rospy.init_node('external_lights')    
    s = rospy.Service('fs20switch', FS20Switch, handle_switch)
    rospy.spin()
