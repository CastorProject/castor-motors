#!/usr/bin/env python
import rospy
import time
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
        self.stopMoveSub = rospy.Subscriber('/stopMove', Bool, self.callbackStopMove)
        self.stopMoveSub = rospy.Subscriber('/interruptMove', Bool, self.callbackInterruptMove)
        return

    def initPublishers(self):
        self.neckPub = rospy.Publisher("/neck_joint_controller/command", Float64, queue_size = 10)
        self.rightShoulderRPub = rospy.Publisher("/right_shoulder_roll_controller/command", Float64, queue_size = 10)
        self.rightShoulderPPub = rospy.Publisher("/right_shoulder_pitch_controller/command", Float64, queue_size = 10)
        self.rightElbowPPub = rospy.Publisher("/right_elbow_pitch_controller/command", Float64, queue_size = 10)
        self.leftShoulderRPub = rospy.Publisher("/left_shoulder_roll_controller/command", Float64, queue_size = 10)
        self.leftShoulderPPub = rospy.Publisher("/left_shoulder_pitch_controller/command", Float64, queue_size = 10)
        self.leftElbowPPub = rospy.Publisher("/left_elbow_pitch_controller/command", Float64, queue_size = 10)
        self.emotionPub = rospy.Publisher('/emotions', String, queue_size = 10)
        return

    def initVariables(self):
        self.motorPosition = Float64()
        self.stopMove = Bool()
        self.movement = String()
        self.emotion = String()
        self.changeMovement = False
        self.mainMovementsDict = {
            "neutral": self.setNeutralPosition,
            "wave": self.wave,
            "pointHead": self.pointHead,
            "pointEyes": self.pointEyes,
            "pointNose": self.pointNose,
            "highfive": self.highfive,
            "rightShoulderR": self.rightShoulderRoll,
            "rightShoulderP": self.rightShoulderPitch,
            "rightElbowP": self.rightElbowPitch,
            "leftShoulderR": self.leftShoulderRoll,
            "leftShoulderP": self.leftShoulderPitch,
            "leftElbowP": self.leftElbowPitch,
            "rightHead": self.rightHead,
            "leftHead": self.leftHead,
            "lvl2Movement1": self.level2Movement1,
            "lvl2Movement2": self.level2Movement2,
            "dance": self.dance,
            "lvl3Movement1": self.level3Movement1,
            "lvl3Movement2": self.level3Movement2,
        }
        return

	#Callbacks
    def callbackMovements(self, msg):
        self.movement = msg.data
        self.changeMovement = True
        return

    def callbackStopMove(self, msg):
        self.changeMovement = msg.data
        return

    def callbackInterruptMove(self, msg):
        self.stopMove = msg.data
        if self.stopMove:
            rospy.loginfo("Stoping movements...")
            self.neckPub.unregister()
            self.rightShoulderRPub.unregister()
            self.rightShoulderPPub.unregister()
            self.rightElbowPPub.unregister()
            self.leftShoulderRPub.unregister()
            self.leftShoulderPPub.unregister()
            self.leftElbowPPub.unregister()
            self.emotionPub.unregister()
        else:
            self.initPublishers()
        return

	#Main Movements
    def setNeutralPosition(self):
        time.sleep(0.25)
        self.motorPosition.data = 0.0
        self.rightShoulderPPub.publish(self.motorPosition)
        time.sleep(0.25)
        self.motorPosition.data = 0.0
        self.rightElbowPPub.publish(self.motorPosition)
        time.sleep(0.25)
        self.motorPosition.data = 0.0
        self.rightShoulderRPub.publish(self.motorPosition)
        time.sleep(0.25)
        self.motorPosition.data = 0.0
        self.leftShoulderPPub.publish(self.motorPosition)
        time.sleep(0.25)
        self.motorPosition.data = 0.0
        self.leftElbowPPub.publish(self.motorPosition)
        time.sleep(0.25)
        self.motorPosition.data = 0.0
        self.leftShoulderRPub.publish(self.motorPosition)
        time.sleep(0.25)
        self.motorPosition.data = 0.0
        self.neckPub.publish(self.motorPosition)
        time.sleep(0.25)
        self.emotion.data = "neutral"
        self.emotionPub.publish(self.emotion)
        self.changeMovement = False
        return

    def wave(self): # left arm
        self.motorPosition.data = 0.0
        self.leftShoulderRPub.publish(self.motorPosition)
        time.sleep(0.25)
        self.motorPosition.data = 1.8
        self.leftShoulderPPub.publish(self.motorPosition)
        time.sleep(0.5)
        self.motorPosition.data = -0.2
        self.leftShoulderRPub.publish(self.motorPosition)
        time.sleep(0.5)
        self.motorPosition.data = 0.3
        self.leftShoulderRPub.publish(self.motorPosition)
        time.sleep(0.5)
        self.motorPosition.data = -0.2
        self.leftShoulderRPub.publish(self.motorPosition)
        time.sleep(0.5)
        self.motorPosition.data = 0.3
        self.leftShoulderRPub.publish(self.motorPosition)
        self.setNeutralPosition()
        return

    #def wave(self):
    #    self.motorPosition.data = 0.0
    #    self.rightShoulderRPub.publish(self.motorPosition)
    #    time.sleep(0.25)
    #    self.motorPosition.data = -1.8
    #    self.rightShoulderPPub.publish(self.motorPosition)
    #    time.sleep(0.25)
    #    self.motorPosition.data = 0.5
    #    self.rightElbowPPub.publish(self.motorPosition)
    #    time.sleep(0.5)
    #    self.motorPosition.data = 0.5
    #    self.rightShoulderRPub.publish(self.motorPosition)
    #    time.sleep(0.5)
    #    self.motorPosition.data = 0.0
    #    self.rightShoulderRPub.publish(self.motorPosition)
    #    time.sleep(0.5)
    #    self.motorPosition.data = 0.5
    #    self.rightShoulderRPub.publish(self.motorPosition)
    #    time.sleep(0.5)
    #    self.motorPosition.data = 0.0
    #    self.rightShoulderRPub.publish(self.motorPosition)
    #    self.setNeutralPosition()
    #    return

    def pointHead(self):
        time.sleep(0.25)
        self.motorPosition.data = -0.3
        self.leftShoulderRPub.publish(self.motorPosition)
        time.sleep(0.25)
        self.motorPosition.data = 1.6
        self.leftShoulderPPub.publish(self.motorPosition)
        time.sleep(0.25)
        self.motorPosition.data = 0.5
        self.leftElbowPPub.publish(self.motorPosition)
        time.sleep(3)
        self.setNeutralPosition()
        return

    def pointEyes(self):
        time.sleep(0.25)
        self.motorPosition.data = 0.7
        self.neckPub.publish(self.motorPosition)
        time.sleep(0.25)
        self.motorPosition.data = -0.5
        self.leftShoulderRPub.publish(self.motorPosition)
        time.sleep(0.25)
        self.motorPosition.data = 1.3
        self.leftShoulderPPub.publish(self.motorPosition)
        time.sleep(0.25)
        self.motorPosition.data = 1.0
        self.leftElbowPPub.publish(self.motorPosition)
        time.sleep(3)
        self.setNeutralPosition()
        return

    def pointNose(self):
        time.sleep(0.25)
        self.motorPosition.data = 0.7
        self.neckPub.publish(self.motorPosition)
        time.sleep(0.25)
        self.motorPosition.data = -0.6
        self.leftShoulderRPub.publish(self.motorPosition)
        time.sleep(0.25)
        self.motorPosition.data = 1.1
        self.leftShoulderPPub.publish(self.motorPosition)
        time.sleep(0.25)
        self.motorPosition.data = 1.0
        self.leftElbowPPub.publish(self.motorPosition)
        time.sleep(3)
        self.setNeutralPosition()
        return

    def pointMouth(self):
        time.sleep(0.25)
        self.motorPosition.data = 1.0
        self.neckPub.publish(self.motorPosition)
        time.sleep(0.25)
        self.motorPosition.data = -0.7
        self.leftShoulderRPub.publish(self.motorPosition)
        time.sleep(0.25)
        self.motorPosition.data = 0.9
        self.leftShoulderPPub.publish(self.motorPosition)
        time.sleep(0.25)
        self.motorPosition.data = 1.0
        self.leftElbowPPub.publish(self.motorPosition)
        self.setNeutralPosition()
        return

    #def highfive(self): #left arm
    #    self.motorPosition.data = 1.8
    #    self.leftShoulderPPub.publish(self.motorPosition)
    #    time.sleep(3)
    #    self.setNeutralPosition()
    #    return

    def highfive(self):
        self.motorPosition.data = 0.0
        self.rightShoulderRPub.publish(self.motorPosition)
        time.sleep(0.25)
        self.motorPosition.data = -1.8
        self.rightShoulderPPub.publish(self.motorPosition)
        time.sleep(0.25)
        self.motorPosition.data = 0.5
        self.rightElbowPPub.publish(self.motorPosition)
        time.sleep(3)
        self.setNeutralPosition()
        return

    def rightShoulderRoll(self):
        self.motorPosition.data = 1.0
        self.rightShoulderRPub.publish(self.motorPosition)
        time.sleep(3)
        self.setNeutralPosition()
        return

    def rightShoulderPitch(self):
        self.motorPosition.data = -1.5
        self.rightShoulderPPub.publish(self.motorPosition)
        time.sleep(3)
        self.setNeutralPosition()
        return

    def rightElbowPitch(self):
        self.motorPosition.data = -1.0
        self.rightElbowPPub.publish(self.motorPosition)
        time.sleep(3)
        self.setNeutralPosition()
        return

    def leftShoulderRoll(self):
        self.motorPosition.data = -1.0
        self.leftShoulderRPub.publish(self.motorPosition)
        time.sleep(3)
        self.setNeutralPosition()
        return

    def leftShoulderPitch(self):
        self.motorPosition.data = 1.5
        self.leftShoulderPPub.publish(self.motorPosition)
        time.sleep(3)
        self.setNeutralPosition()
        return

    def leftElbowPitch(self):
        self.motorPosition.data = 1.0
        self.leftElbowPPub.publish(self.motorPosition)
        time.sleep(3)
        self.setNeutralPosition()
        return

    def rightHead(self):
        self.motorPosition.data = -1.0
        self.neckPub.publish(self.motorPosition)
        time.sleep(3)
        self.setNeutralPosition()
        return

    def leftHead(self):
        self.motorPosition.data = 1.0
        self.neckPub.publish(self.motorPosition)
        time.sleep(3)
        self.setNeutralPosition()
        return

    def level2Movement1(self):
        self.motorPosition.data = -1.6
        self.rightShoulderPPub.publish(self.motorPosition)
        time.sleep(0.25)
        self.motorPosition.data = -1.0
        self.leftShoulderPPub.publish(self.motorPosition)
        time.sleep(4)
        self.setNeutralPosition()
        return

    def level2Movement2(self):
        self.motorPosition.data = -1.5
        self.rightShoulderPPub.publish(self.motorPosition)
        time.sleep(0.25)
        self.motorPosition.data = 1.5
        self.leftShoulderPPub.publish(self.motorPosition)
        time.sleep(4)
        self.setNeutralPosition()
        return

    def dance(self):
        time.sleep(0.25)
        self.motorPosition.data = 1.0
        self.rightShoulderRPub.publish(self.motorPosition)
        time.sleep(0.25)
        self.motorPosition.data = -1.2
        self.rightShoulderPPub.publish(self.motorPosition)
        time.sleep(0.25)
        self.motorPosition.data = -1.0
        self.neckPub.publish(self.motorPosition)
        time.sleep(0.25)
        self.motorPosition.data = 0.0
        self.rightShoulderRPub.publish(self.motorPosition)
        time.sleep(0.25)
        self.motorPosition.data = 0.0
        self.rightShoulderPPub.publish(self.motorPosition)
        time.sleep(0.25)
        self.motorPosition.data = 0.0
        self.neckPub.publish(self.motorPosition)
        time.sleep(0.25)
        self.motorPosition.data = -1.0
        self.leftShoulderRPub.publish(self.motorPosition)
        time.sleep(0.25)
        self.motorPosition.data = 1.0
        self.leftShoulderPPub.publish(self.motorPosition)
        time.sleep(0.25)
        self.motorPosition.data = 1.0
        self.neckPub.publish(self.motorPosition)
        time.sleep(0.25)
        self.motorPosition.data = 0.0
        self.leftShoulderRPub.publish(self.motorPosition)
        time.sleep(0.25)
        self.motorPosition.data = 0.0
        self.leftShoulderPPub.publish(self.motorPosition)
        time.sleep(0.25)
        self.motorPosition.data = 0.0
        self.neckPub.publish(self.motorPosition)
        return

    def level3Movement1(self):
        self.motorPosition.data = -1.5
        self.rightShoulderPPub.publish(self.motorPosition)
        time.sleep(1.5)
        self.motorPosition.data = -1.0
        self.leftShoulderRPub.publish(self.motorPosition)
        time.sleep(1.5)
        self.motorPosition.data = 1.0
        self.neckPub.publish(self.motorPosition)
        time.sleep(3)
        self.setNeutralPosition()
        return

    def level3Movement2(self):
        self.motorPosition.data = -1.5
        self.rightShoulderPPub.publish(self.motorPosition)
        time.sleep(1.5)
        self.motorPosition.data = -1.0
        self.leftShoulderRPub.publish(self.motorPosition)
        time.sleep(1.5)
        self.motorPosition.data = 1.0
        self.neckPub.publish(self.motorPosition)
        time.sleep(3)
        self.setNeutralPosition()
        return
	#Main
    def main(self):
        rospy.loginfo("[%s] dynamixel motor manager node started ok", self.name)
        while not (rospy.is_shutdown()):
            if self.changeMovement:
                try:
                    self.mainMovementsDict[self.movement]()
                except:
                    pass
        return

if __name__=='__main__':
    dynamixelManager = dynamixelManagerNode("dynamixelManager")
    dynamixelManager.main()
