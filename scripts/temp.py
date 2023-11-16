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

# Importing the required libraries
'''
# Team ID:          2405
# Theme:            Luminosity Drone (LD).
# Author List:      Rohit R,Athulkrishna s,Rohit K,Athul Ajaykumar
# Filename:         life_form_detector.py
# Functions:        None
# Global variables: None
'''
from swift_msgs.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int16
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from pid_tune.msg import PidTune
import rospy
import time
import math
from imutils import contours
from skimage import measure
import numpy as np
import imutils
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError
from luminosity_drone.msg import Biolocation

class swift():
	"""docstring for swift"""
	def __init__(self):
		
		rospy.init_node('drone_control')	# initializing ros node with name drone_control

		# This corresponds to your current position of drone. This value must be updated each time in your whycon callback
		# [x,y,z]
		self.drone_position = [0.0,0.0,0.0]	
		self.setpoint_No = 0
		self.bridge = CvBridge()

		# Define the dimensions of the 2D array
		rows = 13  # Number of rows
		cols = 7   # Number of columns (half the range of "y")
		z_value = 27  # Constant value for z

		# Create the 2D array with a snake-like pattern and a gap of 2 in "y"
		self.setpoint = [[-9, -9, 26],
							[-9, 9, 26],
							[-5, 9, 26],
							[-5, -9, 26],
							[-1, -9, 26],
							[-1, 9, 26],
							[3, 9, 26],
							[3, -9, 26],
							[6, -9, 26],
							[6, 9, 26],
							[9, 9, 26],
							[9, -9, 26],
							[9, -9, 26],
							[11, 11, 36.8],
							[11, 11, 37],
							[0, 0, 26],
							[0, 0, 26]]

# whycon marker at the position of the dummy given in the scene. Make the whycon marker associated with position_to_hold dummy renderable and make changes accordingly
		

		#self.setpoint[2]-=1

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

		self.Kp = [18.92, 18.92, 22.58]
		self.Ki = [0.004, 0.004, 0.0]
		self.Kd = [390, 390, 498.9]
   
		#-----------------------Add other required variables for pid here ----------------------------------------------

		self.whcc=[0,0,0]



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





		# Publishing /drone_command, /alt_error, /pitch_error, /roll_error
		self.command_pub = rospy.Publisher('/drone_command', swift_msgs, queue_size=1)
		#------------------------Add other ROS Publishers here-----------------------------------------------------
		self.alt_error_pub = rospy.Publisher('/alt_error', Float64, queue_size=1)
		self.pitch_error_pub = rospy.Publisher('/pitch_error', Float64, queue_size=1)
		self.roll_error_pub = rospy.Publisher('/roll_error', Float64, queue_size=1)
		self.org_pub = rospy.Publisher('/astrobiolocation', Biolocation, queue_size=1)


	#-----------------------------------------------------------------------------------------------------------


		# Subscribing to /whycon/poses, /pid_tuning_altitude, /pid_tuning_pitch, pid_tuning_roll
		rospy.Subscriber('whycon/poses', PoseArray, self.whycon_callback)
		rospy.Subscriber('/pid_tuning_altitude',PidTune,self.altitude_set_pid)
		#-------------------------Add other ROS Subscribers here----------------------------------------------------
		rospy.Subscriber('/pid_tuning_pitch',PidTune,self.pitch_set_pid)
		rospy.Subscriber('/pid_tuning_roll',PidTune,self.roll_set_pid)

		image_topic = "/swift/camera_rgb/image_raw"  # Replace with your actual image topic
		rospy.Subscriber(image_topic, Image, self.image_callback)
		#------------------------------------------------------------------------------------------------------------

		self.arm() # ARMING THE DRONE


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
		
		#--------------------Set the remaining co-ordinates of the drone from msg----------------------------------------------
		self.drone_position[1] = msg.poses[0].position.y
		self.drone_position[2] = 38-msg.poses[0].position.z

		self.whcc[0]=msg.poses[0].position.x
		self.whcc[1]=msg.poses[0].position.y
		self.whcc[2]=msg.poses[0].position.z
	
		#---------------------------------------------------------------------------------------------------------------



	# Callback function for /pid_tuning_altitude
	# This function gets executed each time when /tune_pid publishes /pid_tuning_altitude
	def altitude_set_pid(self,alt):
		self.Kp[2] = alt.Kp * 0.006 # This is just for an example. You can change the ratio/fraction value accordingly
		self.Ki[2] = alt.Ki * 0.0003
		self.Kd[2] = alt.Kd * 0.3
		
	#----------------------------Define callback function like altitide_set_pid to tune pitch, roll--------------

	def pitch_set_pid(self,pitch):
		self.Kp[1] = pitch.Kp * 0.06 # This is just for an example. You can change the ratio/fraction value accordingly
		self.Ki[1] = pitch.Ki * 0.0003
		self.Kd[1] = pitch.Kd * 0.3
		

	def roll_set_pid(self,roll):
		self.Kp[0] = roll.Kp * 0.06 # This is just for an example. You can change the ratio/fraction value accordingly
		self.Ki[0] = roll.Ki * 0.0003
		self.Kd[0] = roll.Kd * 0.3
		
	def image_callback(self,immm):
		image = self.bridge.imgmsg_to_cv2(immm, "bgr8")
		cv2.imshow("Image Title", image)
		gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
		blurred = cv2.GaussianBlur(gray, (11, 11), 0)
		thresh = cv2.threshold(blurred, 200, 255, cv2.THRESH_BINARY)[1]
		thresh = cv2.erode(thresh, None, iterations=2)
		thresh = cv2.dilate(thresh, None, iterations=4)
		labels = measure.label(thresh, connectivity=2, background=0)
		mask = np.zeros(thresh.shape, dtype="uint8")
		for label in np.unique(labels):
				# If this is the background label, ignore it
				if label == 0:
					continue

				# Otherwise, construct the label mask and count the number of pixels
				labelMask = np.zeros(thresh.shape, dtype="uint8")
				labelMask[labels == label] = 255
				numPixels = cv2.countNonZero(labelMask)

				# If the number of pixels in the component is sufficiently large, then add it to our mask of "large blobs"
				if numPixels > 100:
					mask = cv2.add(mask, labelMask)
			
		cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		cnts = imutils.grab_contours(cnts)
		if len(cnts) > 0:
			cnts = contours.sort_contours(cnts)[0]

			num_leds = len(cnts)

			if(num_leds==2):
				self.organism_type = 'alien_a'
			elif(num_leds==3):
				self.organism_type='alien_b'
			elif(num_leds==4):
				self.organism_type='alien_c'
			# print(f"No. of LEDs detected: {num_leds}\n")
			# Calculate the center of the image
			total_cX = 0
			total_cY = 0

			image_height, image_width = image.shape[:2]
			center_x = image_width / 2
			center_y = image_height / 2

			for i, c in enumerate(cnts):
				area = float(cv2.contourArea(c))
				M = cv2.moments(c)
				if M["m00"] != 0:
					cX = float(M["m10"] / M["m00"])
					cY = float(M["m01"] / M["m00"])
				else:
					cX, cY = 0.0, 0.0
				


				total_cX += cX
				total_cY += cY

				#print(f"Centroid #{i + 1}: ({format(cX, '.10f')}, {format(cY, '.10f')})")
				#print(f"Area #{i + 1}: {format(area, '.1f')}")







			if num_leds > 0:
				# Calculate the average centroid
				average_cX = total_cX / num_leds
				average_cY = total_cY / num_leds
				# print(f"Average Centroid: ({format(average_cX, '.10f')}, {format(average_cY, '.10f')})")

				# Calculate the offset of the average centroid from the center
				offset_x = average_cX - center_x
				offset_y = average_cY - center_y
				# print(f"Offset from center: ({format(offset_x, '.10f')}, {format(offset_y, '.10f')})")






			# print(self.whcc)
			if(self.setpoint_No!=15 and self.setpoint_No<12):
				print("organism_type found now deep searching")
				self.setpoint[15][0]=self.whcc[0]
				self.setpoint[15][1]=self.whcc[1]
				self.setpoint[15][2]=self.whcc[2]

				self.setpoint_No=15

			elif(self.setpoint_No!=13):
				self.setpoint[15][0]=self.setpoint[15][0]+offset_x*0.0005
				self.setpoint[15][1]=self.setpoint[15][1]+offset_y*0.0005

				self.setpoint[15][2]=28

				if(abs(offset_x)<10 and abs(offset_y)<10):

					self.setpoint_No=13

					self.biolocation_msg = Biolocation()
					self.biolocation_msg.organism_type = self.organism_type
					self.biolocation_msg.whycon_x = self.whcc[0]
					self.biolocation_msg.whycon_y = self.whcc[1]
					self.biolocation_msg.whycon_z = self.whcc[2]

					# Publish the message
					print(self.biolocation_msg,"Now prepair disarm")
					self.org_pub.publish(self.biolocation_msg)
	 


	#----------------------------------------------------------------------------------------------------------------------

	def pid(self):
	#-----------------------------Write the PID algorithm here--------------------------------------------------------------

	# Steps:
	# 	1. Compute error in each axis. eg: error[0] = self.drone_position[0] - self.setpoint[0] ,where error[0] corresponds to error in x...
	#	2. Compute the error (for proportional), change in error (for derivative) and sum of errors (for integral) in each axis. Refer "Understanding PID.pdf" to understand PID equation.
	#	3. Calculate the pid output required for each axis. For eg: calcuate self.out_roll, self.out_pitch, etc.
	#	4. Reduce or add this computed output value on the avg value ie 1500. For eg: self.cmd.rcRoll = 1500 + self.out_roll. LOOK OUT FOR SIGN (+ or -). EXPERIMENT AND FIND THE CORRECT SIGN
	#	5. Don't run the pid continously. Run the pid only at the a sample time. self.sampletime defined above is for this purpose. THIS IS VERY IMPORTANT.
	#	6. Limit the output value and the final command value between the maximum(2000) and minimum(1000)range before publishing. For eg : if self.cmd.rcPitch > self.max_values[1]:
	#																														self.cmd.rcPitch = self.max_values[1]
	#	7. Update previous errors.eg: self.prev_error[1] = error[1] where index 1 corresponds to that of pitch (eg)
	#	8. Add error_sum
		


		#print(self.drone_position)
		#print(self.setpoint)

		self.alt_error = -(self.drone_position[2]- (38-(self.setpoint[self.setpoint_No][2]-1.6)))
		self.cmd.rcThrottle=int(1545 + self.alt_error * self.Kp[2] + (self.alt_error - self.prev_alt_error)*self.Kd[2]+(self.sum_alt_error+self.alt_error) * self.Ki[2])
		if self.cmd.rcThrottle>2000:
			self.cmd.rcThrottle=2000
		if self.cmd.rcThrottle<1000:
			self.cmd.rcThrottle=1000


		integral_limit = 100  
		if self.sum_alt_error > integral_limit:
			self.sum_alt_error = integral_limit
		elif self.sum_alt_error < -integral_limit:
			self.sum_alt_error = -integral_limit

		# Check if control signal is saturated and reset the integral term
		if self.cmd.rcThrottle == 2000 or self.cmd.rcThrottle == 1000:
			self.sum_alt_error = 0

		self.prev_alt_error =self.alt_error
		self.sum_alt_error=self.sum_alt_error+self.alt_error





		self.pitch_error = (self.drone_position[1]- self.setpoint[self.setpoint_No][1])
		self.cmd.rcPitch=int(1500 + self.pitch_error * self.Kp[1] + (self.pitch_error - self.prev_pitch_error)*self.Kd[1]+(self.sum_pitch_error+self.pitch_error )* self.Ki[1])
		if self.cmd.rcPitch>2000:
			self.cmd.rcPitch=2000
		if self.cmd.rcPitch<1000:
			self.cmd.rcPitch=1000

		self.prev_pitch_error =self.pitch_error
		self.sum_pitch_error=self.sum_pitch_error+self.pitch_error




		self.roll_error = -(self.drone_position[0]- self.setpoint[self.setpoint_No][0])
		self.cmd.rcRoll=int(1500 + self.roll_error * self.Kp[0] + (self.roll_error - self.prev_roll_error)*self.Kd[0]+(self.sum_roll_error+self.roll_error) * self.Ki[0])
		if self.cmd.rcRoll>2000:
			self.cmd.rcRoll=2000
		if self.cmd.rcRoll<1000:
			self.cmd.rcRoll=1000

		self.prev_roll_error =self.roll_error
		self.sum_roll_error=self.sum_roll_error+self.roll_error

		if (abs(self.whcc[2] - self.setpoint[self.setpoint_No][2]) <= 1 and abs(self.whcc[1] - self.setpoint[self.setpoint_No][1]) <= 0.49 and abs(self.whcc[0] - self.setpoint[self.setpoint_No][0]) <= 0.49 and self.setpoint_No < 14):


			
			self.setpoint_No += 1
			if(self.setpoint_No>13):
				self.cmd.rcThrottle=1580
				self.cmd.rcPitch=1500
				self.cmd.rcRoll=1500
				self.command_pub.publish(self.cmd)
				rospy.sleep(1)

				self.disarm()
				self.disarm_v=1
				print("drone disarmed")
			print(self.setpoint[self.setpoint_No][:])
			print(self.whcc)







	#------------------------------------------------------------------------------------------------------------------------
		if(self.disarm_v==1):		
			self.cmd.rcRoll = 1500
			self.cmd.rcPitch = 1500
			self.cmd.rcYaw = 1500
			self.cmd.rcThrottle = 999
		self.command_pub.publish(self.cmd)
		
		self.alt_error_pub.publish(self.alt_error-1.7)
		self.pitch_error_pub.publish(self.pitch_error)
		self.roll_error_pub.publish(self.roll_error)







if __name__ == '__main__':

	swift_drone = swift()
	r = rospy.Rate(30) #specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
	while not rospy.is_shutdown():
		swift_drone.pid()
		r.sleep()