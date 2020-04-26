#!/usr/bin/env python

# ROS import
import rospy
from wifi_mapping.srv import WiFiSignal

import rssi

def handle_wifi_signals(req):
	# Select the correct interface (depends on the machine)
	interface = "wlp3s0"
	rssi_scanner = rssi.RSSI_Scan(interface)

	# List of access points to scan
	ssids = [req.ssid]

	ap_info = rssi_scanner.getAPinfo(networks=ssids, sudo=True)

	if type(ap_info) is not bool:
		minimo = float("inf")
		for ap in ap_info:
			if abs(ap["signal"])<minimo:
				minimo = ap["signal"]
		
		return(minimo)
	else:
		return(0)

def wifi_signals_srv():
	rospy.init_node("wifi_signals_srv")
	srv = rospy.Service("wifi_signals", WiFiSignal, handle_wifi_signals)
	print("Ready to return WiFi signal")
	rospy.spin()

if __name__ == "__main__":
	wifi_signals_srv()