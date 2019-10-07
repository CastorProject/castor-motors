#!/usr/bin/env python
import rospy
import time
import pygame
from std_msgs.msg import Float64
from std_msgs.msg import String
from std_msgs.msg import Bool

class dynamixelManagerNode(object):
	def __init__(self, name):
		self.name = name
		rospy.init_node(self.name)
		self.rate = rospy.Rate(0.5)
		self.initSubscribers()
		self.initPublishers()
		self.initVariables()
		return

	def initSubscribers(self):
		self.movementSub = rospy.Subscriber('/movements', String, self.callbackMovements)
		self.motorPositionSub = rospy.Subscriber('/dMotorsPosition', Float64, self.callbackMotorsPosition)
		self.dynamixelMotorSub = rospy.Subscriber('/dMotors', String, self.callbackDynamixelMotor)
		return
		

	def initPublishers(self):
		self.neckPub = rospy.Publisher("/neck_joint_controller/command", Float64, queue_size = 10)
		self.rightShoulderRPub = rospy.Publisher("/right_shoulder_roll_controller/command", Float64, queue_size = 10)
		self.rightShoulderPPub = rospy.Publisher("/right_shoulder_pitch_controller/command", Float64, queue_size = 10)
		self.rightElbowPPub = rospy.Publisher("/right_elbow_pitch_controller/command", Float64, queue_size = 10)
		self.leftShoulderRPub = rospy.Publisher("/left_shoulder_roll_controller/command", Float64, queue_size = 10)
		self.leftShoulderPPub = rospy.Publisher("/left_shoulder_pitch_controller/command", Float64, queue_size = 10)
		self.leftElbowPPub = rospy.Publisher("/left_elbow_pitch_controller/command", Float64, queue_size = 10)
		self.wordsPub = rospy.Publisher("/speaker", String, queue_size = 10)
		self.stopTalkPub = rospy.Publisher('/stopTalk', Bool, queue_size = 10)
		self.emotionPub = rospy.Publisher('/emotions', String, queue_size = 10)
		return

	def initVariables(self):
		self.motorPosition = Float64()
		self.words = String()
		self.stopTalk = Bool()
		self.emotion = String()
		self.changeMovement = False	
		self.changeMotor = False	
		self.mainMovementsDict = {
						"greet": self.greeting,
						"Lbye": self.Lbye,
						"greet_bye": self.greeting_bye, 
						"nicetomeet": self.nicetomeet,
						"howru": self.how_r_u,
						"fine": self.fine,
						"guess": self.guess,
						"tryagain": self.play,
						"play": self.play,
						"nice": self.nice,
						"neutral": self.setNeutralPosition			
					 }
		self.dynamixelMotorsDict = {
					"neck": self.moveNeck,
					"rightShoulderR": self.moveRSR,
					"rightShoulderP": self.moveRSP,
					"rightElbowP": self.moveREP,
					"leftShoulderR": self.moveLSR,
					"leftShoulderP": self.moveLSP,
					"leftElbowP": self.moveLEP
				     }
		
		return
	
	#Callbacks
	def callbackMovements(self, msg):
		self.movement = msg.data
		self.changeMovement = True
		return
	
	def callbackMotorsPosition(self, msg):
		self.motorPosition = msg.data
		return

	def callbackDynamixelMotor(self, msg):
		self.dynamixelMotor = msg.data
		self.changeMotor = True
		return
	
	#Main Movements
	def setNeutralPosition(self):
		time.sleep(0.5)
		self.motorPosition.data = 0.0
		self.neckPub.publish(self.motorPosition)
		time.sleep(0.5)
		self.motorPosition.data = 0.1
		self.rightShoulderRPub.publish(self.motorPosition)
		time.sleep(0.5)
		self.motorPosition.data = -2.0
		self.rightShoulderPPub.publish(self.motorPosition)
		time.sleep(0.5)
		self.motorPosition.data = -0.1
		self.rightElbowPPub.publish(self.motorPosition)
		time.sleep(0.5)
		self.motorPosition.data = 0.0
		self.leftShoulderRPub.publish(self.motorPosition)
		time.sleep(0.5)
		self.motorPosition.data = 0.0
		self.leftShoulderPPub.publish(self.motorPosition)
		time.sleep(0.5)
		self.motorPosition.data = 0.0
		self.leftElbowPPub.publish(self.motorPosition)
		return

	def play(self):
		self.stopTalk.data = False
		self.stopTalkPub.publish(self.stopTalk)
		self.emotion.data = "talk"
		self.emotionPub.publish(self.emotion)
		pygame.mixer.init()
		pygame.mixer.music.load("/home/pi/Documents/quieresjugar.mp3")
		pygame.mixer.music.play()
		while pygame.mixer.music.get_busy() == True:
    			continue
		self.stopTalk.data = True
		self.stopTalkPub.publish(self.stopTalk)
		return
				
	def guess(self):
		self.stopTalk.data = False
		self.stopTalkPub.publish(self.stopTalk)
		self.emotion.data = "talk"
		self.emotionPub.publish(self.emotion)
		pygame.mixer.init()
		pygame.mixer.music.load("/home/pi/Documents/adivina.mp3")
		pygame.mixer.music.play()
		while pygame.mixer.music.get_busy() == True:
    			continue
		self.stopTalk.data = True
		self.stopTalkPub.publish(self.stopTalk)
		return

	def fine(self):
		self.stopTalk.data = False
		self.stopTalkPub.publish(self.stopTalk)
		self.emotion.data = "talk"
		self.emotionPub.publish(self.emotion)
		pygame.mixer.init()
		pygame.mixer.music.load("/home/pi/Documents/estoymuybien.mp3")
		pygame.mixer.music.play()
		while pygame.mixer.music.get_busy() == True:
    			continue
		self.stopTalk.data = True
		self.stopTalkPub.publish(self.stopTalk)
		return

	def nice(self):
		self.stopTalk.data = False
		self.stopTalkPub.publish(self.stopTalk)
		self.emotion.data = "talk"
		self.emotionPub.publish(self.emotion)
		pygame.mixer.init()
		pygame.mixer.music.load("/home/pi/Documents/muybien.mp3")
		pygame.mixer.music.play()
		while pygame.mixer.music.get_busy() == True:
    			continue
		self.stopTalk.data = True
		self.stopTalkPub.publish(self.stopTalk)
		return
		
	def tryagain(self):
		self.stopTalk.data = False
		self.stopTalkPub.publish(self.stopTalk)
		self.emotion.data = "talk"
		self.emotionPub.publish(self.emotion)
		pygame.mixer.init()
		pygame.mixer.music.load("/home/pi/Documents/intentaotravez.mp3")
		pygame.mixer.music.play()
		while pygame.mixer.music.get_busy() == True:
    			continue
		self.stopTalk.data = True
		self.stopTalkPub.publish(self.stopTalk)
		return

	def how_r_u(self):
		self.stopTalk.data = False
		self.stopTalkPub.publish(self.stopTalk)
		self.emotion.data = "talk"
		self.emotionPub.publish(self.emotion)
		pygame.mixer.init()
		pygame.mixer.music.load("/home/pi/Documents/comoestas.mp3")
		pygame.mixer.music.play()
		while pygame.mixer.music.get_busy() == True:
    			continue
		self.stopTalk.data = True
		self.stopTalkPub.publish(self.stopTalk)
		return

	def nicetomeet(self):
		self.stopTalk.data = False
		self.stopTalkPub.publish(self.stopTalk)
		self.emotion.data = "talk"
		self.emotionPub.publish(self.emotion)
		pygame.mixer.init()
		pygame.mixer.music.load("/home/pi/Documents/mealegro.mp3")
		pygame.mixer.music.play()
		while pygame.mixer.music.get_busy() == True:
    			continue
		self.stopTalk.data = True
		self.stopTalkPub.publish(self.stopTalk)
		return

	def Lbye(self):
                time.sleep(0.5)
                self.motorPosition.data = -0.2
                self.leftShoulderRPub.publish(self.motorPosition)
		time.sleep(0.5)
		self.motorPosition.data = 1.5
		self.leftShoulderPPub.publish(self.motorPosition)
		time.sleep(0.5)
		self.motorPosition.data = 0.5
		self.leftElbowPPub.publish(self.motorPosition)
		self.stopTalk.data = False
		self.stopTalkPub.publish(self.stopTalk)
		self.emotion.data = "talk"
		self.emotionPub.publish(self.emotion)
		pygame.mixer.init()
		pygame.mixer.music.load("/home/pi/Documents/despedir.mp3")
		pygame.mixer.music.play()
		while pygame.mixer.music.get_busy() == True:
			time.sleep(0.5)
			self.motorPosition.data = 0.5
			self.leftShoulderRPub.publish(self.motorPosition)
			time.sleep(0.5)
			self.motorPosition.data = 0.0
			self.leftShoulderRPub.publish(self.motorPosition)
    			continue
		self.stopTalk.data = True
		self.stopTalkPub.publish(self.stopTalk)
		self.setNeutralPosition()
		return
		
	def greeting_bye(self):
                time.sleep(0.5)
                self.motorPosition.data = -0.2
                self.leftShoulderRPub.publish(self.motorPosition)
                time.sleep(0.5)
                self.motorPosition.data = 1.5
                self.leftShoulderPPub.publish(self.motorPosition)
                time.sleep(0.5)
                self.motorPosition.data = 0.5
                self.leftElbowPPub.publish(self.motorPosition)
                time.sleep(0.5)
                self.motorPosition.data = 0.5
                self.leftShoulderRPub.publish(self.motorPosition)
                time.sleep(0.5)
                self.motorPosition.data = 0.0
                self.leftShoulderRPub.publish(self.motorPosition)
                time.sleep(0.5)
                self.motorPosition.data = 0.5
                self.leftShoulderRPub.publish(self.motorPosition)
                time.sleep(0.5)
                self.motorPosition.data = 0.0
                self.leftShoulderRPub.publish(self.motorPosition)
		self.setNeutralPosition()
		return

	def greeting(self):
                time.sleep(0.5)
                self.motorPosition.data = -0.2
                self.leftShoulderRPub.publish(self.motorPosition)
                time.sleep(0.5)
                self.motorPosition.data = 1.5
                self.leftShoulderPPub.publish(self.motorPosition)
                time.sleep(0.5)
                self.motorPosition.data = 0.5
                self.leftElbowPPub.publish(self.motorPosition)
		self.stopTalk.data = False
		self.stopTalkPub.publish(self.stopTalk)
		self.emotion.data = "talk"
		self.emotionPub.publish(self.emotion)
		pygame.mixer.init()
		pygame.mixer.music.load("/home/pi/Documents/saludo.mp3")
		pygame.mixer.music.play()
		while pygame.mixer.music.get_busy() == True:
			time.sleep(0.5)
			self.motorPosition.data = 0.5
			self.leftShoulderRPub.publish(self.motorPosition)
			time.sleep(0.5)
			self.motorPosition.data = 0.0
			self.leftShoulderRPub.publish(self.motorPosition)
    			continue
		self.stopTalk.data = True
		self.stopTalkPub.publish(self.stopTalk)
		self.setNeutralPosition()
		return

	#Dynamixel Motors
	def moveNeck(self):
		self.neckPub.publish(self.motorPosition)
		return

	def moveRSR(self):
		self.rightShoulderRPub.publish(self.motorPosition)
		return

	def moveRSP(self):
		self.rightShoulderPPub.publish(self.motorPosition)
		return

	def moveREP(self):
		self.rightElbowPPub.publish(self.motorPosition)
		return

	def moveLSR(self):
		self.leftShoulderRPub.publish(self.motorPosition)
		return

	def moveLSP(self):
		self.leftShoulderPPub.publish(self.motorPosition)
		return

	def moveLEP(self):
		self.leftElbowPPub.publish(self.motorPosition)
		return

	#Main
	def main(self):
		rospy.loginfo("[%s] dynamixel motor manager node started ok", self.name)
		while not (rospy.is_shutdown()):
			if self.changeMovement:
				self.mainMovementsDict[self.movement]()
				self.changeMovement = False
			elif self.changeMotor:
				self.dynamixelMotorsDict[self.dynamixelMotor]()
				self.changeMotor = False
		return
		

if __name__=='__main__':
	dynamixelManager = dynamixelManagerNode("dynamixelManager")		
	dynamixelManager.main()

