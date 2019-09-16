#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import Float64



class arm_publisher(object):
	def __init__(self):
		rospy.init_node('arm_publisher')
		self.rate = rospy.Rate(0.5)
		self.topics = [	
				"/neck_joint_controller/command",
				"/right_shoulder_roll_controller/command",
				"/right_shoulder_pitch_controller/command",
				"/right_elbow_pitch_controller/command",
				"/left_shoulder_roll_controller/command",
				"/left_shoulder_pitch_controller/command",
				"/left_elbow_pitch_controller/command"
			      ]
		self.data = [Float64() for i in range(7)]

		self.pub = [rospy.Publisher(t, Float64, queue_size = 10) for t in self.topics]

		
		self.head_state = 0
		self.left_arm_state = 0
		self.right_arm_state = 0


	def set_init(self):
		d = Float64()
		d.data = 0
		for p in self.pub:
			p.publish(d)
		d.data = -1
		self.pub[1].publish(d)
		d.data =  1
		self.pub[4].publish(d)
	
	def get_head_trajectory(self):
		d = Float64()
		if self.head_state == 0:
			d.data = 0.5
			self.head_state = 1				
		elif self.head_state == 1:
			d.data = 0
			self.head_state = 2
		elif self.head_state == 2:
			d.data = -0.5
			self.head_state = 0
		else:
			d.data = 0
			self.head_state = 1				
		return d
		
	
	def get_arm_trajectory(self):
		d = Float64()
		if self.head_state == 0:
			d.data = 1.0
			self.head_state = 1				
		elif self.head_state == 1:
			d.data = 0 
			self.head_state = 2
		elif self.head_state == 2:
			d.data = 1.0
			self.head_state = 0
		else:
			d.data = 0
			self.head_state = 1				
		return d
	
			
	
	def loop(self):
		self.set_init()
		time.sleep(5)
		while not rospy.is_shutdown():
			d = self.get_head_trajectory()
			self.pub[0].publish(d)
			#self.rate.sleep()
			d = self.get_arm_trajectory()
			self.pub[2].publish(d)
			#self.rate.sleep()
			self.pub[3].publish(d)
			#self.rate.sleep()
			self.pub[5].publish(d)
			#self.rate.sleep()
			self.pub[6].publish(d)								
			self.rate.sleep()

if __name__=='__main__':

	a = arm_publisher()		

	a.loop()

