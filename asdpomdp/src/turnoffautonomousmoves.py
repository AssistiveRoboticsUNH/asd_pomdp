'''
turnoffautonomousmoves.py
Madison Clark-Turner
12/17/2016

Turn off the NAO's autonomous moves after speech-recognition has been activated
'''

import sys
import signal
from naoqi import ALProxy

robotIp = "192.168.1.12"

if len(sys.argv) <= 1:
	print "Usage python turnoffautonomousmoves.py robotIP"
else:
	robotIp = sys.argv[1]

try:
	autoMove = ALProxy("ALAutonomousMoves", robotIP, 9559)
except Exception,e:
	print "Could not create proxy to ALAutonomousMoves"
	print "Error was: ",e

autoMove.setExpressiveListeningEnabled(False);
