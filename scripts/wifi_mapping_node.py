#!/usr/bin/env python

"""
	This code uses the information from /map and the service 
	wifi_signals_srv to make a map of the WiFi strength
"""

# General imports
import sys

# Import ROS dependencies
import rospy

# Import msgs
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String

# Import srv
from wifi_signals_clt import wifi_signal_client as wifi_client

robot_pose = None
marker_id = 0
min_strength = float("inf")
max_strength = float("-inf")
m_ant = 1

marker_array = MarkerArray()

def odomCb(msg):
	global robot_pose
	robot_pose = msg.pose.pose

def publishMarkers(strength_msg):
	global marker_id, min_strength, max_strength, m_ant, marker_array
	if strength_msg.z is not 0:
		pub_marker_array = rospy.Publisher("wifi_markers", MarkerArray, queue_size=10)
		marker2publish = Marker()
		marker2publish.header.frame_id = "odom"
		marker2publish.ns = "wifi_strength"
		marker2publish.id = marker_id
		marker_id = marker_id+1
		marker2publish.type = 3
		marker2publish.action = 0
		marker2publish.pose.position.x = strength_msg.x
		marker2publish.pose.position.y = strength_msg.y
		marker2publish.pose.orientation.x = 0
		marker2publish.pose.orientation.y = 0
		marker2publish.pose.orientation.z = 0
		marker2publish.pose.orientation.w = 1
		marker2publish.scale.x = 0.05
		marker2publish.scale.y = 0.05
		if abs(strength_msg.z) < 40:
			marker2publish.color.r = 0
			marker2publish.color.g = 1
			marker2publish.color.b = 0
			marker2publish.color.a = 1
		elif abs(strength_msg.z) < 50:
			marker2publish.color.r = 0.9
			marker2publish.color.g = 0.5
			marker2publish.color.b = 0
			marker2publish.color.a = 1
		else:
			marker2publish.color.r = 1
			marker2publish.color.g = 0
			marker2publish.color.b = 0
			marker2publish.color.a = 1

		if min_strength != float("inf") and max_strength != float("-inf") and min_strength != max_strength:
			m_ant = 0.8/(max_strength-min_strength)

		print("min=%f, max=%f, m=%f"%(min_strength,max_strength,m_ant))
		if strength_msg.z > max_strength:
			max_strength = strength_msg.z
			marker2publish.scale.z = 1 # abs(1.0/float(strength_msg.z))*15
			marker2publish.pose.position.z = float(marker2publish.scale.z)/float(2)
			marker_array.markers.append(marker2publish)
			resizeMarkers(marker_id)
		elif strength_msg.z < min_strength:
			min_strength = strength_msg.z
			marker2publish.scale.z = 0.2 # abs(1.0/float(strength_msg.z))*15
			marker2publish.pose.position.z = float(marker2publish.scale.z)/float(2)
			marker_array.markers.append(marker2publish)
			resizeMarkers(marker_id)
		else:
			if marker2publish.id==0:
				marker2publish.scale.z = 0.5  # abs(1.0/float(strength_msg.z))*15
			else:
				marker2publish.scale.z = 0.2 + m_ant*(strength_msg.z-min_strength) # abs(1.0/float(strength_msg.z))*15
			
			marker2publish.pose.position.z = float(marker2publish.scale.z)/float(2)
			marker_array.markers.append(marker2publish)


		# marker2publish.scale.z = abs(1.0/float(strength_msg.z))*15
		pub_marker_array.publish(marker_array)

def resizeMarkers(mid):
	global marker_array, min_strength, max_strength, m_ant
	m = (0.8/(max_strength-min_strength))
	for marker in marker_array.markers:
		if marker.id != mid:
			print("m=%f, m_ant=%f, scale=%f"%(m,m_ant,marker.scale.z))
			marker.scale.z = m/m_ant * marker.scale.z
			print("scale=%f"%(marker.scale.z))
			marker.pose.position.z = float(marker.scale.z)/float(2)
		if marker.scale.z == 0:
			marker_array.markers.remove(marker)

		'''
		if marker_id == 1:
			min_strength = strength_msg.z
			max_strength = strength_msg.z
			marker2publish.scale.z = 1 #abs(1.0/float(strength_msg.z))*15
			marker2publish.pose.position.z = float(marker2publish.scale.z)/float(2)
			# pub_marker.publish(marker2publish)
			marker_array.markers.append(marker2publish)
		elif marker_id == 2:
			if strength_msg.z > min_strength:
				min_strength = strength_msg.z
				marker2publish.scale.z = 0.2 #abs(1.0/float(strength_msg.z))*15
				m_ant = 0.8/float(max_strength-min_strength)				
			
			elif strength_msg.z > max_strength:
				max_strength = strength_msg.z
				marker2publish.scale.z = 1 #abs(1.0/float(strength_msg.z))*15
				m_ant = 0.8/float(max_strength-min_strength)				

			marker2publish.pose.position.z = float(marker2publish.scale.z)/float(2)
			# pub_marker.publish(marker2publish)
			marker_array.markers.append(marker2publish)
		else:
			if strength_msg.z > max_strength:
				max_strength = strength_msg.z
				marker2publish.scale.z = 1 #abs(1.0/float(strength_msg.z))*15
				resizeMarkers()
			elif strength_msg.z < min_strength:
				marker2publish.scale.z = 0.2 #abs(1.0/float(strength_msg.z))*15
				min_strength = strength_msg.z
				resizeMarkers()
			else:
				marker2publish.scale.z = 0.5 #abs(1.0/float(strength_msg.z))*15
				marker2publish.pose.position.z = float(marker2publish.scale.z)/float(2)
				#pub_marker.publish(marker2publish)
				marker_array.markers.append(marker2publish)

		# print(strength_msg.z)
		# print(marker2publish.scale.z)
		# marker2publish.pose.position.z = float(marker2publish.scale.z)/float(2)
		# pub_marker.publish(marker2publish)
		pub_marker_array.publish(marker_array)
	else:
		pass

def resizeMarkers():
	global min_strength, max_strength, m_ant, marker_array
	# pub_marker = rospy.Publisher("wifi_markers", Marker, queue_size=10)
	# marker2edit = Marker()
	print(len(marker_array.markers))
	for marker in marker_array.markers:
		#marker2edit.header.frame_id = "odom"
		#marker2edit.ns = "wifi_strength"
		#marker2edit.id = i
		#marker2edit.action = 0
		marker.scale.z = (0.8/(max_strength-min_strength))/m_ant * marker.scale.z #abs(1.0/float(strength_msg.z))*15
		marker.pose.position.z = float(marker.scale.z)/float(2)
		# pub_marker.publish(marker2edit)
		marker_array.markers.append(marker)
		
	m_ant = 0.8/float(max_strength-min_strength)
'''

def goalStatusCb(msg):
	global getSignal 
	if msg.data == "success":
		getSignal = True


def saveData(strength_msg):
	f = open("/home/joseluis/Documents/wifi_data.txt", "a+")
	f.write("%f %f %f \n" % (strength_msg.x, strength_msg.y, strength_msg.z))
	f.close()

def wifi_mapping(ssid):
	global robot_pose, getSignal
	getSignal = False;
	''' Initiate node '''
	rospy.init_node("wifi_mapping")

	''' Subscribers '''
	sub_odom = rospy.Subscriber("/odom", Odometry, odomCb)
	sub_goal_status = rospy.Subscriber("/goal_status", String, goalStatusCb)

	''' Publishers '''
	# This publisher sends the signal strength (z) at position (x,y)
	pub_strength = rospy.Publisher("wifi_strength", Point, queue_size=10)
	pub_command = rospy.Publisher("command", String, queue_size=10)

	f = open("/home/joseluis/Documents/wifi_data.txt", "a+")
	f.write("\n *** NEW DATA *** \n")
	f.close()

	while not rospy.is_shutdown():
		# print(robot_pose)
		if getSignal:
			wifi_strength = wifi_client(ssid)
			strength_msg = Point()
			if robot_pose is not None:
				strength_msg.x = robot_pose.position.x
				strength_msg.y = robot_pose.position.y
			else:
				strength_msg.x = 0
				strength_msg.y = 0
		
			strength_msg.z = wifi_strength
			pub_strength.publish(strength_msg)
			saveData(strength_msg)
			publishMarkers(strength_msg)
			getSignal = False
			command_msg = String()
			command_msg.data = "next"
			pub_command.publish(command_msg)
		# print("[WIFI_MAPPING] Msg sent")



if __name__ == "__main__":
	
	if len(sys.argv) > 2:
		ssid = str(sys.argv[1])
	else:
		print("Specify SSID as argv[1]")
		sys.exit(1)

	try:
		wifi_mapping(ssid)
	except rospy.ROSInterruptException:
		pass
