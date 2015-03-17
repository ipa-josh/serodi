#!/usr/bin/python
import os, time

#make sure adb is working and login works without password!
#tested with ssh server for android

def shutdown():
	os.system("adb shell reboot -p") #does not work on my device

if __name__ == '__main__':
	while True:
		os.system("adb shell am force-stop com.icecoldapps.sshserver")
		os.system("adb shell am start -n com.icecoldapps.sshserver/.viewStart")
		os.system("adb forward tcp:2222 tcp:2222")
		time.sleep(5)
		os.system("(ssh -R 9090:localhost:9090 serodi@localhost -p 2222 & );ssh -R 38300:localhost:38300 serodi@localhost -p 2222")
		time.sleep(0.2)
