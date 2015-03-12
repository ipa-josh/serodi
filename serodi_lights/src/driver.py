#!/usr/bin/env python
import os
import sys
 
#install python-hid
import usb.core
import usb.util
 
from time import sleep
import random
import struct

import rospy

class FS20Driver:
	def __init__(self, mbed_vendor_id, mbed_product_id):
		self.connected = False
		self.mbed_vendor_id = mbed_vendor_id
		self.mbed_product_id = mbed_product_id

	def connect(self):
	    self.connected = False
	    self.hid_device = usb.core.find(idVendor=self.mbed_vendor_id,idProduct=self.mbed_product_id)
	    
	    if not self.hid_device:
		return "No device connected"
	    else:
		if self.hid_device.is_kernel_driver_active(0):
		    try:
		        self.hid_device.detach_kernel_driver(0)
		    except usb.core.USBError as e:
		        return "Could not detatch kernel driver: %s" % str(e)
		try:
		    self.hid_device.set_configuration()
		    self.hid_device.reset()
		except usb.core.USBError as e:
		    return "Could not set configuration: %s" % str(e)
		
		endpoint = self.hid_device[0][(0,0)][0]  
	    self.connected = True
	    return None    

	def send(self, housecode, addr_fs20, on):
		#clear
		self.hid_device.read(endpoint.bEndpointAddress, 8)

		self.hid_device.write(1, self.create_sendcmd(housecode, addr_fs20, on))
		data = self.hid_device.read(endpoint.bEndpointAddress, 2000) #wait 2s
		return self.parse_resp(data)

	def code2bytes(self, housecode, exp_len):
		if len(housecode)!=exp_len:
			raise Exception('housecode2bytes', 'wrong input')
		c1 = lambda x: int(x[1])-1 + 4*(int(x[0])-1)
		c2 = lambda x: 16*c1(x[0:])+c1(x[2:])
		if exp_len==8:
			return [c2(housecode), c2(housecode[4:])]
		return c2(housecode)

	def create_sendcmd(self, housecode, addr_fs20, on):
		data = [0x0] * 8
		data[0] = 0x01 #report id
		data[1] = len(data)-2 #size
		data[2] = 0xF1 #Hauscode, Adresse und Sendebefehl einmalig senden.

		#HC1, HC2, Adr, Bef, Erw
		(data[3], data[4]) = code2bytes(housecode,8)
		data[5] = code2bytes(addr_fs20,4)
		data[6] = 0x11 if on else 0x00

		return data

	def parse_resp(self, data):
		if len(data)<2: return "too short"
		if data[1]+2!=len(data): return "wrong number of bytes (1)"
		if data[1]+2!=5: return "wrong number of bytes (2)"
		if data[2]!=0xA0: return "answer id wrong"

		if data[3]==0: return None
		if data[3]==2: return "unknown command id"
		if data[3]==3: return "wrong lenth of req."
		if data[3]==4: return "canceled"
		if data[3]==5: return "nothing to stop"

		return "some error"

if __name__ == "__main__":
    vendor = 6383
    product = 57365

    if len(sys.argv)!=4:
		print "[driver.py] housecode address [True/False]"
	
    driver = FS20Driver(vendor, product)
    
    resp = driver.connect()
    if resp!=None:
        print resp
        exit(1)
		
	housecode = sys.argv[1]
	addr_fs20 = sys.argv[2]
	on = bool(sys.argv[3])
    print driver.send(housecode, addr_fs20, on)
