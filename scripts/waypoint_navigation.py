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

from swift_msgs.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int16
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from pid_tune.msg import PidTune
import rospy
import time


class swift():
	"""docstring for swift"""
	def __init__(self):
		
		rospy.init_node('drone_control')	# initializing ros node with name drone_control

		# This corresponds to your current position of drone. This value must be updated each time in your whycon callback
		# [x,y,z]
		self.drone_position = [0.0,0.0,0.0]	

		# [x_setpoint, y_setpoint, z_setpoint]
		self.setpoints = [[0, 0, 23], [2, 0, 23], [2, 2, 23], [2, 2, 25], [-5, 2, 25], [-5, -3, 25], [-5, -3, 21], [7, -3, 21], [7, 0, 21], [0, 0, 19]]
		self.setpoint = [0, 0, 23]
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


		#initial setting of Kp, Kd and ki for [roll, pitch, throttle]. eg: self.Kp[2] corresponds to Kp value in throttle axis
		#after tuning and computing corresponding PID parameters, change the parameters

		self.Kp = [20,40,80]
		self.Ki = [0,0,0.01] #0.001
		self.Kd = [44,22,465]
   
		#-----------------------Add other required variables for pid here ----------------------------------------------
		
		self.error = [0.0, 0.0 ,0.0]
		self.deri = [0.0, 0.0, 0.0]
		self.error_sum = [0.0, 0.0, 0.0]
		self.prev_error = [0.0, 0.0, 0.0]
		self.correct = [0.0, 0.0, 0.0]

		self.max_values = [2000,2000,2000]
		self.min_values = [1000,1000,1000]






		# Hint : Add variables for storing previous errors in each axis, like self.prev_error = [0,0,0] where corresponds to [pitch, roll, throttle]		
		#    	 Add variables for limiting the values like self.max_values = [2000,2000,2000] corresponding to [roll, pitch, throttle]
		#													self.min_values = [1000,1000,1000] corresponding to [pitch, roll, throttle]
		#																	You can change the upper limit and lower limit accordingly. 
		#----------------------------------------------------------------------------------------------------------

		# # This is the sample time in which you need to run pid. Choose any time which you seem fit. Remember the stimulation step time is 50 ms
		self.sample_time = 0.06# in seconds
		self.last_time = 0.0
		self.current_time=0





		# Publishing /drone_command, /alt_error, /pitch_error, /roll_error
		self.command_pub = rospy.Publisher('/drone_command', swift_msgs, queue_size=1)
		#------------------------Add other ROS Publishers here-----------------------------------------------------
		self.alt_error_pub = rospy.Publisher('/alt_error', Float64, queue_size=1)
		self.pit_error_pub = rospy.Publisher('/pitch_error', Float64, queue_size=1)
		self.roll_error_pub = rospy.Publisher('/roll_error', Float64, queue_size=1)






	#-----------------------------------------------------------------------------------------------------------


		# Subscribing to /whycon/poses, /pid_tuning_altitude, /pid_tuning_pitch, pid_tuning_roll
		rospy.Subscriber('whycon/poses', PoseArray, self.whycon_callback)
		rospy.Subscriber('/pid_tuning_altitude',PidTune,self.altitude_set_pid)
		#-------------------------Add other ROS Subscribers here----------------------------------------------------
		rospy.Subscriber('/pid_tuning_pitch',PidTune,self.pitch_set_pid)
		rospy.Subscriber('/pid_tuning_roll',PidTune,self.roll_set_pid)



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
		self.drone_position[2] = msg.poses[0].position.z



	
		#---------------------------------------------------------------------------------------------------------------



	# Callback function for /pid_tuning_altitude
	# This function gets executed each time when /tune_pid publishes /pid_tuning_altitude
	def altitude_set_pid(self,alt):
		self.Kp[2] = alt.Kp  # This is just for an example. You can change the ratio/fraction value accordingly
		self.Ki[2] = alt.Ki*0.01
		self.Kd[2] = alt.Kd 
		
	#----------------------------Define callback function like altitide_set_pid to tune pitch, roll--------------
	def pitch_set_pid(self,pit):
		self.Kp[1] = pit.Kp  # This is just for an example. You can change the ratio/fraction value accordingly
		self.Ki[1] = pit.Ki 
		self.Kd[1] = pit.Kd 

	def roll_set_pid(self,roll):
		self.Kp[0] = roll.Kp  # This is just for an example. You can change the ratio/fraction value accordingly
		self.Ki[0] = roll.Ki 
		self.Kd[0] = roll.Kd 





	#----------------------------------------------------------------------------------------------------------------------

	def pid(self):
		# Roll error
		self.error[0] = (self.setpoint[0]-self.drone_position[0])
		self.cmd.rcRoll = int(1500+self.error[0]*self.Kp[0]+(
			self.error[0]-self.prev_error[0])*self.Kd[0]+self.error_sum[0]*self.Ki[0])
		if(self.cmd.rcRoll > 1600):
			self.cmd.rcRoll = 1600
		if(self.cmd.rcRoll < 1400):
			self.cmd.rcRoll = 1400
		self.prev_error[0] = self.error[0]
		if(self.error[0] < 0.15 and self.error[0] > -0.15):
			self.error_sum[0] += self.error[0]
		else:
			self.error_sum[0] = 0
		self.roll_error_pub.publish(self.error[0])
		# Pitch error
		self.error[1] = -(self.setpoint[1]-self.drone_position[1])
		self.cmd.rcPitch = int(1500+self.error[1]*self.Kp[1]+(
			self.error[1]-self.prev_error[1])*self.Kd[1]+self.error_sum[1]*self.Ki[1])
		if(self.cmd.rcPitch > 1600):
			self.cmd.rcPitch = 1600
		if(self.cmd.rcPitch < 1400):
			self.cmd.rcPitch = 1400
		self.prev_error[1] = self.error[1]
		if(self.error[1] < 0.15 and self.error[1] > -0.15):
			self.error_sum[1] += self.error[1]
		else:
			self.error_sum[1] = 0
		self.pit_error_pub.publish(self.error[1])
		# Throttle error
		self.error[2] = -(self.setpoint[2]-self.drone_position[2])
		self.cmd.rcThrottle = int(1580+self.error[2]*self.Kp[2]+(
			self.error[2]-self.prev_error[2])*self.Kd[2]+self.error_sum[2]*self.Ki[2])
		if(self.cmd.rcThrottle > 1800):
			self.cmd.rcThrottle = 1800
		if(self.cmd.rcThrottle < 1000):
			self.cmd.rcThrottle = 1000
		self.prev_error[2] = self.error[2]
		if(self.error[2] > -0.5 and self.error[2] < 0.5):
			self.error_sum[2] += self.error[2]
		else:
			self.error_sum[2] = 0
		self.alt_error_pub.publish(self.error[2])
		self.command_pub.publish(self.cmd)



def next():
    swift_drone.pid()
    r = rospy.Rate(30)
    waypoints = swift_drone.setpoints
    i = 0
    while i != 10:
        swift_drone.setpoint = waypoints[i]
        while True:
            swift_drone.pid()
            if(swift_drone.drone_position[0] < waypoints[i][0]+0.15 and swift_drone.drone_position[0] > waypoints[i][0]-0.15
               and swift_drone.drone_position[1] < waypoints[i][1]+0.15 and swift_drone.drone_position[1] > waypoints[i][1]-0.15 and
               swift_drone.drone_position[2] < waypoints[i][2]+0.15 and swift_drone.drone_position[2] > waypoints[i][2] - 0.15):
                print(swift_drone.drone_position)
                break
            r.sleep()

        i += 1


if __name__ == '__main__':
    t = time.time()
    while time.time() - t < 5:
        pass
    swift_drone = swift()
    # specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
    next()
    while not rospy.is_shutdown():
        swift_drone.pid()



