#!/usr/bin/env python

import rospy
from serodi_lights.srv import *
from driver import FS20Driver
import time

driver = None
def handle_switch(req):
    global driver

    resp = driver.send(req.housecode, req.addr_fs20, req.on)

    r = FS20SwitchResponse()
    r.success = (resp==None)
    r.error_msg = str(resp)
    return r

if __name__ == "__main__":
    rospy.init_node('external_lights')

    vendor = rospy.get_param("~vendor_id", 6383)
    product = rospy.get_param("~product_id", 57365)
    driver = FS20Driver(vendor, product)
    
    resp = driver.connect()
    if resp!=None:
        rospy.logerr(resp)
        time.sleep(1)
        exit(1)

    s = rospy.Service('fs20switch', FS20Switch, handle_switch)
    rospy.spin()
