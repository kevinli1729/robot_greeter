#!/usr/bin/env python
import sys, getopt     # for getting and parsing command line arguments
import time
import math
import threading
import rospy
import smach
import smach_ros

#from std_msgs.msg import Float64, Float64MultiArray, MultiArrayDimension, Bool, Int32

import numpy as np

class ReadyState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["done", "exit"])
    def execute(self, userdata):
        print "***************"
        print "Executing ReadyState."
        print "***************"
        if rospy.is_shutdown():
            return "exit"
        else:
            return "done"

class Scan(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["found", "leave", "exit"])
    def execute(self, userdata):
        print "***************"
        print "Executing Scan."
        print "***************"
		#runs the program to move neck around while simultaniously detecting faces
        v_yes = raw_input("Found human?")
        if rospy.is_shutdown():
            return "exit"
        elif v_yes == "y":
            return "found"
        else:
            return "leave"

class Center(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["stay", "gone", "exit"])

    def execute(self, userdata):
        print "***************"
        print "Executing Center."
        print "***************"
		#runs program to center dreamer's cameras on the human
        v_yes = raw_input("Human stay in sight?")
        if rospy.is_shutdown():
            return "exit"
        elif v_yes == "y":
            return "stay"
        else:
            return "gone"

class FaceRec(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["done", "exit"])

    def execute(self, userdata):
        print "***************"
        print "Executing FaceRec."
        print "***************"
		#runs the program to recognize the face.
		#note: need to store name in a global variable

        if rospy.is_shutdown():
            return "exit"
        else:
            return "done"

class HandShake(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["done", "exit"])

    def execute(self, userdata):
        print "***************"
        print "Executing HandShake."
        print "***************"
		#execute handshake program

        if rospy.is_shutdown():
            return "exit"
        else:
            return "done"

class RemoveHuman(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["done", "exit"])

    def execute(self, userdata):
        print "***************"
        print "Executing RemoveHuman."
        print "***************"
		#execute removal

        if rospy.is_shutdown():
            return "exit"
        else:
            return "done"

class Greet:
	#Scan, Center, FaceRec, HandShake, HumanMovesAway
    def __init__(self):
        self = self;

    def createFSM(self):
        # define the states
        toReadyState = ReadyState()
       	toScan = Scan()
       	toCenter = Center()
       	toFaceRec = FaceRec()
       	toHandShake = HandShake()
       	toRemoveHuman = RemoveHuman()

        # wire the states into a FSM
        self.fsm = smach.StateMachine(outcomes=['exit'])
        with self.fsm:
            smach.StateMachine.add("GoToReady", toReadyState, 
                transitions={'done':'Scan',
                             'exit':'exit'})
            smach.StateMachine.add("Scan", toScan, 
                transitions={'found':'Center',
                			'leave':'RemoveHuman',
                             'exit':'exit'})
            smach.StateMachine.add("Center", toCenter, 
                transitions={'stay':'FaceRec',
                             'gone':'Scan',
                             'exit':'exit'})
            smach.StateMachine.add("FaceRec", toFaceRec, 
                transitions={'done':'HandShake',
                             'exit':'exit'})
            smach.StateMachine.add("HandShake", toHandShake, 
                transitions={'done':'Scan',
                             'exit':'exit'})
            smach.StateMachine.add("RemoveHuman", toRemoveHuman, 
                transitions={'done':'Scan',
                             'exit':'exit'})
    def run(self):
        self.createFSM()

        # Create and start the introspection server
        sis = smach_ros.IntrospectionServer('server_name', self.fsm, '/SM_ROOT')
        sis.start()

        index = raw_input("Start demo? Y/n\n")
        if index == "N" or index == "n":
            return

        outcome = self.fsm.execute()

        rospy.spin()  # just to prevent this node from exiting
        sis.stop()


# Main method
if __name__ == "__main__":
    rospy.init_node('Greet', anonymous=True)
    demo = Greet()
    demo.run()