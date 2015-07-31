#!/usr/bin/env python
import rospy
import controlit_goal_generators
import time
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import Char
from std_msgs.msg import Float64MultiArray
import math

scan_mode = "";
scan_dir = True;

def stringBack(data):
	#print data.data
	global scan_mode
	scan_mode = data.data


def boolBack(data):
	global scan_dir
	scan_dir = data.data

def controller():
	pub1 = rospy.Publisher("/dreamer_controller/neck/goal_position", Float64MultiArray, queue_size=10)
	pub2 = rospy.Publisher("/dreamer_controller/neck/eye_goal_position", Float64MultiArray, queue_size=10)
	pub3 = rospy.Publisher("fail", Char, queue_size = 10)
	rospy.init_node('robot_head_controller', anonymous=True)
	#rospy.init_node("scan_dir_listener", anonymous = True)
	rate = rospy.Rate(10) # 10hz

	tau = 3.1415926535798932 * 2

	global scan_mode
	global scan_dir
	while not rospy.is_shutdown():
		rospy.Subscriber("chatter", String, stringBack)
		rospy.Subscriber("chatter2", Bool, boolBack)

		#print "Testing if publishing a char works"

		print "Waiting for facial recognition to start."
		while scan_mode != "scan" and not rospy.is_shutdown():
			#print scan_mode
			rate.sleep()
			#rospy.spinOnce()
		print "Scanning mode initiated."
		
		startTime = time.time()
		eye_yaw = 0
		neck_yaw = 0
		
		while scan_mode == "scan" and not rospy.is_shutdown():
		#while not rospy.is_shutdown():
			curtime = time.time() - startTime
			xValue = curtime / 20 * tau
			eye_yaw = math.sin(xValue) * 0.59
			neck_yaw = math.sin(xValue) * 1.4

			neckArray = Float64MultiArray();
			eyeArray = Float64MultiArray();

			neckArray.data.append(0)
			neckArray.data.append(neck_yaw)
			neckArray.data.append(0)
			neckArray.data.append(0)

			eyeArray.data.append(0)
			eyeArray.data.append(eye_yaw)
			eyeArray.data.append(eye_yaw)

			pub1.publish(neckArray)
			pub2.publish(eyeArray)

			print str(neck_yaw) + ' ' + str(eye_yaw)

			rate.sleep()

		eyeArray = Float64MultiArray();
		eyeArray.data.append(0)
		eyeArray.data.append(0)
		eyeArray.data.append(0)
		pub2.publish(eyeArray)

		print "Scanning Horizontally."
		fail_test = Char()
		fail_test.data = 'g'
		pub3.publish(fail_test)
		while scan_mode == "focus_h" and fail_test != 'f' and not rospy.is_shutdown():
			if scan_dir:
				neck_yaw -= 0.005
			else:
				neck_yaw += 0.005

			if abs(neck_yaw) > 1.4:
				fail_test.data = 'f'
			else:
				neckArray = Float64MultiArray();
				neckArray.data.append(0)
				neckArray.data.append(neck_yaw)
				neckArray.data.append(0)
				neckArray.data.append(0)
				pub1.publish(neckArray)

			pub3.publish(fail_test)
			rate.sleep()

		print "Scanning Vertically."
		neck_pitch = 0;
		while scan_mode == "focus_v" and fail_test != 'f' and not rospy.is_shutdown():
			if scan_dir:
				neck_pitch -= 0.005
			else:
				neck_pitch += 0.005

			if neck_pitch < -0.11 or neck_pitch > 0.64:
				fail_test.data = 'f'
			else:
				neckArray = Float64MultiArray();
				neckArray.data.append(0)
				neckArray.data.append(neck_yaw)
				neckArray.data.append(0)
				neckArray.data.append(neck_pitch)
				pub1.publish(neckArray)

			pub3.publish(fail_test)
			rate.sleep()

if __name__ == '__main__':
	try:
		controller()
	except rospy.ROSInterruptException:
		pass