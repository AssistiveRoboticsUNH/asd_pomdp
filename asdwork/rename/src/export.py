#!/usr/bin/env python
import rosbag
import sys
import cv2
from cv_bridge import CvBridge
import numpy as np
import time
import os.path, os

if len(sys.argv) <= 2:
  print "\n".join(("USAGE\t%s <bag> <image topic 1> [image topic 2] [image topic 3] ..." % sys.argv[0], "\tnote: the FULL name should be used...  if the images are from a compressed transport, the topic name should end in \"/compressed\""))
  sys.exit(1)

def toS(t):
    return float(t.secs)+(float(t.nsecs) / 1000000000.0)
def toMS(t):
    return toS(t) * 1000.0

#TODO naming (or pathing) output appropriately for multiple image topics

filename = sys.argv[1]
image_names = [sys.argv[i] for i in range(2,len(sys.argv))]

count=0
filepath = "%s" % sys.argv[1]
filepath = filepath[:-4]

if not os.path.exists(filepath):
        os.mkdir(filepath)

get_over_it = CvBridge()

for topic, msg, stamp in rosbag.Bag(filename).read_messages():
    if topic in image_names:
        epoch = toS(stamp)
        outname = "%s/image_%f.jpg" % (filepath, epoch) #format stamp for naming -- this might be broken
        npimage = get_over_it.imgmsg_to_cv2(msg)
        cv2.imwrite(outname, npimage)
        os.utime(os.path.join(os.getcwd(),outname),(int(epoch),int(epoch)))
        print("exporting %s ..." % outname)
        count=count+1

print("exported %d images" % count)
