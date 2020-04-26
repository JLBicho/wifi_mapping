#!/usr/bin/env python
# Get WiFi signals information

# ROS dependencies
import rospy
from std_msgs.msg import Int32

# Import the module
import rssi

def wifi_publisher():
	# Initializes node
	rospy.init_node("wifi_signals", anonymous=True)

	# Publisher
	pub = rospy.Publisher("wifi_strength", Int32, queue_size=10)
	
	print("Initializing WiFi signal...")

	# Select the correct interface (depends on the machine)
	interface = "wlp3s0"
	rssi_scanner = rssi.RSSI_Scan(interface)

	# List of access points to scan
	ssids = ['eduroam']

	while not rospy.is_shutdown():
		ap_info = rssi_scanner.getAPinfo(networks=ssids, sudo=True)

		if type(ap_info) is not bool:
			print(ap_info)
			minimo = float("inf")
			for ap in ap_info:
				if abs(ap["signal"])<minimo:
					minimo = ap["signal"]
			
			# print(minimo)
			pub.publish(minimo)


if __name__=="__main__":
	try:
		wifi_publisher()
	except rospy.ROSInterruptException:
		pass