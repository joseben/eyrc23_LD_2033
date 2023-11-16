#!/usr/bin/env python3

'''

This python file runs a ROS-node of name drone_control which holds the position of Swift-Drone on the given dummy.
This node publishes and subsribes the following topics:

		PUBLICATIONS			SUBSCRIPTIONS
		/drone_command			/whycon/poses
		/alt_error				/pid_tuning_altitude
		/pitch_error			/pid_tuning_pitch
		/roll_error				/pid_tuning_roll
					
								

Rather than using different variables, use list. eg : self.setpoint = [1,2,3], where index corresponds to x,y,z ...rather than defining self.x_setpoint = 1, self.y_setpoint = 2
CODE MODULARITY AND TECHNIQUES MENTIONED LIKE THIS WILL HELP YOU GAINING MORE MARKS WHILE CODE EVALUATION.	
'''

from swift_msgs.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int16
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from pid_tune.msg import PidTune
import rospy
import time
import cv2
from imutils import contours
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError
from skimage import measure
import numpy as np
import imutils
from sensor_msgs.msg import Image
from luminosity_drone.msg import Biolocation

class swift():
	"""docstring for swift"""
	def __init__(self):
		
		rospy.init_node('drone_control')	# initializing ros node with name drone_control

		# [x,y,z]
		self.drone_position = [0.0,0.0,0.0]	

		self.waypoints = 0
		self.bridge = CvBridge()


		#2D Map
		self.setpoint = [[-9, -9, 25],[-9, 9, 25],[-5, 9, 25],[-5, -9, 25],[-1, -9, 25],[-1, 9, 25],[3, 9, 25],[3, -9, 25],[6, -9, 25],[6, 9, 25],[9, 9, 25],[9, -9, 25],[9, -9, 25],[11, 11, 37.5],[11, 11, 37.5],[0, 0, 25],[0, 0, 25]]

		self.current_setpoint_index = 0
		self.max_setpoint_distance = 0.1  # Adjust this value as needed
		self.is_at_setpoint = False

		#Declaring a cmd of message type swift_msgs and initializing values
		self.cmd = swift_msgs()
		self.cmd.rcRoll = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcThrottle = 1500
		self.cmd.rcAUX1 = 1500
		self.cmd.rcAUX2 = 1500
		self.cmd.rcAUX3 = 1500
		self.cmd.rcAUX4 = 1500

		self.disarm_v=0
		#initial setting of Kp, Kd and ki for [roll, pitch, throttle]. eg: self.Kp[2] corresponds to Kp value in throttle axis
		#after tuning and computing corresponding PID parameters, change the parameters

		self.Kp = [19, 19, 22.6]
		self.Ki = [0.004, 0.004, 0.0]
		self.Kd = [391, 391, 500]
   
		#-----------------------Add other required variables for pid here ----------------------------------------------

		self.error = [0.0, 0.0 ,0.0]
		self.deri = [0.0, 0.0, 0.0]
		self.error_sum = [0.0, 0.0, 0.0]
		self.prev_error = [0.0, 0.0, 0.0]
		self.correct = [0.0, 0.0, 0.0]

		self.max_values = [2000,2000,2000]
		self.min_values = [1000,1000,1000]

		self.cord=[0,0,0]


		self.alt_error = 0.0
		self.prev_alt_error = 0.0
		self.sum_alt_error = 0.0
		self.min_throttle = 1000
		self.max_throttle = 2000
		self.pitch_error = 0.0
		self.prev_pitch_error = 0.0
		self.sum_pitch_error = 0.0
		self.min_pitch= 1200
		self.max_pitch = 1800
		self.roll_error = 0.0
		self.prev_roll_error = 0.0
		self.sum_roll_error = 0.0

		self.min_roll = 1200
		self.max_roll = 1800



		# Hint : Add variables for storing previous errors in each axis, like self.prev_error = [0,0,0] where corresponds to [pitch, roll, throttle]		#		 Add variables for limiting the values like self.max_values = [2000,2000,2000] corresponding to [roll, pitch, throttle]
		#													self.min_values = [1000,1000,1000] corresponding to [pitch, roll, throttle]
		#																	You can change the upper limit and lower limit accordingly. 
		#----------------------------------------------------------------------------------------------------------

		# # This is the sample time in which you need to run pid. Choose any time which you seem fit. Remember the stimulation step time is 50 ms
		self.sample_time = 0.033 # in seconds

		self.last_time = 0.0
		self.current_time=0



		# Publishing /drone_command, /alt_error, /pitch_error, /roll_error
		self.command_pub = rospy.Publisher('/drone_command', swift_msgs, queue_size=1)
		#------------------------Add other ROS Publishers here-----------------------------------------------------
		self.alt_error_pub = rospy.Publisher('/alt_error', Float64, queue_size=1)
		self.pitch_error_pub = rospy.Publisher('/pitch_error', Float64, queue_size=1)
		self.roll_error_pub = rospy.Publisher('/roll_error', Float64, queue_size=1)
		self.org_pub = rospy.Publisher('/astrobiolocation', Biolocation, queue_size=1)

		rospy.Subscriber('whycon/poses', PoseArray, self.whycon_callback)
		rospy.Subscriber('/pid_tuning_altitude',PidTune,self.altitude_set_pid)
		#-------------------------Add other ROS Subscribers here----------------------------------------------------
		rospy.Subscriber('/pid_tuning_pitch',PidTune,self.pitch_set_pid)
		rospy.Subscriber('/pid_tuning_roll',PidTune,self.roll_set_pid)
		img = "/swift/camera_rgb/image_raw"  
		rospy.Subscriber(img, Image, self.img_func)
		self.arm() 


	# Disarming condition of the drone
	def disarm(self):
		self.cmd.rcAUX4 = 1100

		self.command_pub.publish(self.cmd)
		rospy.sleep(1)


	# Arming condition of the drone : Best practise is to disarm and then arm the drone.
	def arm(self):

		self.disarm()

		self.cmd.rcRoll = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcThrottle = 1000
		self.cmd.rcAUX4 = 1500
		self.command_pub.publish(self.cmd)	# Publishing /drone_command
		rospy.sleep(1)



	# Whycon callback function
	# The function gets executed each time when /whycon node publishes /whycon/poses 
	def whycon_callback(self,msg):
		self.drone_position[0] = msg.poses[0].position.x
		self.drone_position[1] = msg.poses[0].position.y
		self.drone_position[2] = 38-msg.poses[0].position.z
		self.cord[0]=msg.poses[0].position.x
		self.cord[1]=msg.poses[0].position.y
		self.cord[2]=msg.poses[0].position.z
	
		#---------------------------------------------------------------------------------------------------------------

	def altitude_set_pid(self,alt):
		self.Kp[2] = alt.Kp  # This is just for an example. You can change the ratio/fraction value accordingly
		self.Ki[2] = alt.Ki*0.01
		self.Kd[2] = alt.Kd 
		
	def pitch_set_pid(self,pit):
		self.Kp[1] = pit.Kp  # This is just for an example. You can change the ratio/fraction value accordingly
		self.Ki[1] = pit.Ki 
		self.Kd[1] = pit.Kd 

	def roll_set_pid(self,roll):
		self.Kp[0] = roll.Kp  # This is just for an example. You can change the ratio/fraction value accordingly
		self.Ki[0] = roll.Ki 
		self.Kd[0] = roll.Kd 


		
	def img_func(self,immm):
		image = self.bridge.imgmsg_to_cv2(immm, "bgr8")
		gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
		blurred = cv2.GaussianBlur(gray, (11, 11), 0)
		thresh = cv2.threshold(blurred, 200, 255, cv2.THRESH_BINARY)[1]
		thresh = cv2.erode(thresh, None, iterations=2)
		thresh = cv2.dilate(thresh, None, iterations=4)
		labels = measure.label(thresh, connectivity=2, background=0)
		mask = np.zeros(thresh.shape, dtype="uint8")
		for label in np.unique(labels):
				if label == 0:
					continue
				labelMask = np.zeros(thresh.shape, dtype="uint8")
				labelMask[labels == label] = 255
				numPixels = cv2.countNonZero(labelMask)
				if numPixels > 100:
					mask = cv2.add(mask, labelMask)
			
		contors = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		contors = imutils.grab_contours(contors)
		if len(contors) > 0:
			contors = contours.sort_contours(contors)[0]
			num_leds = len(contors)

			sumcX = 0
			sumcY = 0

			height, width = image.shape[:2]
			cntX = width / 2
			cntY = height / 2

			for i, c in enumerate(contors):
				area = float(cv2.contourArea(c))
				M = cv2.moments(c)
				if M["m00"] != 0:
					centeroidX = float(M["m10"] / M["m00"])
					centeroidY = float(M["m01"] / M["m00"])
				else:
					centeroidX, centeroidY = 0.0, 0.0
		
				sumcX = sumcX + centeroidX
				sumcY = sumcY + centeroidY


			if num_leds > 0:
				avgCx = sumcX / num_leds
				avgCy = sumcY / num_leds
				offset_x = avgCx - cntX
				offset_y = avgCy - cntY
		
			
			if(num_leds==2):
				self.organism_type = 'alien_a'
			elif(num_leds==3):
				self.organism_type='alien_b'
			elif(num_leds==4):
				self.organism_type='alien_c'


			if(self.waypoints!=15 and self.waypoints<12):
				print("Localising")
				self.setpoint[15][0]=self.cord[0]
				self.setpoint[15][1]=self.cord[1]
				self.setpoint[15][2]=self.cord[2]

				self.waypoints=15

			elif(self.waypoints!=13):
				self.setpoint[15][0]=self.setpoint[15][0]+offset_x*0.0005
				self.setpoint[15][1]=self.setpoint[15][1]+offset_y*0.0005
				self.setpoint[15][2]=25

				if(abs(offset_x)<10 and abs(offset_y)<10):

					self.waypoints=13
					self.biolocation_msg = Biolocation()
					self.biolocation_msg.organism_type = self.organism_type
					self.biolocation_msg.whycon_x = self.cord[0]
					self.biolocation_msg.whycon_y = self.cord[1]
					self.biolocation_msg.whycon_z = self.cord[2]
					print(self.biolocation_msg,"\n Heading to Research Station")
					self.org_pub.publish(self.biolocation_msg)
	 


	#----------------------------------------------------------------------------------------------------------------------

	def pid(self):
#***********************************************************************
		self.roll_error = -(self.drone_position[0]- self.setpoint[self.waypoints][0])
		self.cmd.rcRoll=int(1500 + self.roll_error * self.Kp[0] + (self.roll_error - self.prev_roll_error)*self.Kd[0]+(self.sum_roll_error+self.roll_error) * self.Ki[0])
		if self.cmd.rcRoll>2000:
			self.cmd.rcRoll=2000
		if self.cmd.rcRoll<1000:
			self.cmd.rcRoll=1000

		self.prev_roll_error =self.roll_error
		self.sum_roll_error=self.sum_roll_error+self.roll_error
#***********************************************************************
		# Calculate altitude error
		self.alt_error = -(self.drone_position[2] - (38 - (self.setpoint[self.waypoints][2] - 1.6)))

		# Proportional, Integral, Derivative (PID) control
		self.cmd.rcThrottle = int(1545 + self.alt_error * self.Kp[2] + 
								(self.alt_error - self.prev_alt_error) * self.Kd[2] +
								(self.sum_alt_error + self.alt_error) * self.Ki[2])

		# Saturate control signal
		self.cmd.rcThrottle = max(1000, min(self.cmd.rcThrottle, 2000))

		# Limit integral term
		integral_limit = 100
		self.sum_alt_error = max(-integral_limit, min(self.sum_alt_error, integral_limit))

		# Reset integral term if control signal is saturated
		if self.cmd.rcThrottle == 2000 or self.cmd.rcThrottle == 1000:
			self.sum_alt_error = 0

		# Update previous altitude error
		self.prev_alt_error = self.alt_error

		# Update sum of altitude error
		self.sum_alt_error += self.alt_error

#***********************************************************************
		self.pitch_error = (self.drone_position[1]- self.setpoint[self.waypoints][1])
		self.cmd.rcPitch=int(1500 + self.pitch_error * self.Kp[1] + (self.pitch_error - self.prev_pitch_error)*self.Kd[1]+(self.sum_pitch_error+self.pitch_error )* self.Ki[1])
		if self.cmd.rcPitch>2000:
			self.cmd.rcPitch=2000
		if self.cmd.rcPitch<1000:
			self.cmd.rcPitch=1000

		self.prev_pitch_error =self.pitch_error
		self.sum_pitch_error=self.sum_pitch_error+self.pitch_error
		
		self.command_pub.publish(self.cmd)
		self.alt_error_pub.publish(self.alt_error-1.7)
		self.pitch_error_pub.publish(self.pitch_error)
		self.roll_error_pub.publish(self.roll_error)


		if (
			0.2 >= abs(self.cord[2] - self.setpoint[self.waypoints][2]) >= 0
			and 0.3 >= abs(self.cord[1] - self.setpoint[self.waypoints][1]) >= 0
			and 0.3 >= abs(self.cord[0] - self.setpoint[self.waypoints][0]) >= 0
			and self.waypoints < 14
		):
			
			self.waypoints = self.waypoints + 1
			if(self.waypoints>13 and self.waypoints<16):
				self.cmd.rcThrottle=1580
				self.cmd.rcPitch=1500
				self.cmd.rcRoll=1500
				self.command_pub.publish(self.cmd)

				print("Disarmed")
			print(self.cord)

		self.command_pub.publish(self.cmd)
		self.alt_error_pub.publish(self.alt_error-1.7)
		self.pitch_error_pub.publish(self.pitch_error)
		self.roll_error_pub.publish(self.roll_error)


if __name__ == '__main__':

	swift_drone = swift()
	r = rospy.Rate(30) 
	while not rospy.is_shutdown():
		swift_drone.pid()
		r.sleep()