#!/usr/bin/env python

import sys
import rospy
from wifi_mapping.srv import WiFiSignal

def wifi_signal_client(ssid):
	rospy.wait_for_service("wifi_signals")
	try:
		wifi_signal = rospy.ServiceProxy("wifi_signals", WiFiSignal)
		resp1 = wifi_signal(ssid)
		return resp1.strength
	except rospy.ServiceException, e:
		print("Service call failed: %s"%e)

def usage():
	return("%s"%sys.argv[0])

if __name__ == "__main__":
	if len(sys.argv) == 2:
		ssid = str(sys.argv[1])
	else:
		print usage()
		sys.exit(1)

	print("Requesting WiFi Signal from %s"%ssid)
	print("%d"%(wifi_signal_client(ssid)))
	print("Continue")