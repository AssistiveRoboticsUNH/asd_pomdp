#!/usr/bin/env python
'''
disableexpressivelistening.py
Madison Clark-Turner
1/26/2016

A service to turn off the NAO's autonomous moves after speech-recognition
has been activated
'''

import sys
import signal
from naoqi import ALProxy
from std_srvs.srv import Empty, EmptyResponse
import rospy

def disableExpressiveListening(req):
	
	robotIP = "192.168.1.12"
	
	try:
		autoMove = ALProxy("ALAutonomousMoves", robotIP, 9559)
	except Exception,e:
		print "Could not create proxy to ALAutonomousMoves"
		print "Error was: ",e

	autoMove.setExpressiveListeningEnabled(False);
	
	print "Expressive Listening Disabled"
	return EmptyResponse()

def disableExpressiveListening_server():
	rospy.init_node('disable_expressive_listening_server')
	s = rospy.Service('/disable_expressive_listening', Empty, disableExpressiveListening)
	print "Ready to disable expressive listening."
	rospy.spin()

if __name__ == "__main__":
	disableExpressiveListening_server()
