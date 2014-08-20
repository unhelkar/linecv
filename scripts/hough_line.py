#!/usr/bin/env python
import roslib
roslib.load_manifest('linecv')

import time
import math
import numpy 

import sys
import rospy
import cv2

from rospy.numpy_msg import numpy_msg
from std_msgs.msg import String, Float64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class line_finder:

  def __init__(self):

    # parameters
    self.cThresh1 = 140
    self.cThresh2 = 180
    self.cApertureSize = 4

    self.rho 	= 1
    self.theta 	= numpy.pi/180

    self.houghThresh = 30
    self.houghMinLen = 30
    self.houghMaxGap = 5

    self.minAngle = math.radians(360)
    self.maxAngle = math.radians(0)

    self.image_pub = rospy.Publisher("oup_hough",Image)
    self.canny_pub = rospy.Publisher("oup_canny",Image)
    self.nline_pub = rospy.Publisher("oup_nline",Float64)
    rospy.init_node('line_finder', anonymous=True)

    cv2.namedWindow("Image window", 1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("inp_image",Image,self.callback)

  # reject diagonals beyond a certain angle
  def checkHoughs(self,x1,y1,x2,y2):
    # calculate line parameters
    l = ( (x1-x2)**2 + (y1-y2)**2 )**0.5
    m = abs((y2-y1)/(x1-x2)) # slope
    angle = math.atan(m)
    # print l
    return True

    # rule based rejection of lines detected by Hough transform
    oneVertex = [0,0]
    twoVertex = [0,0]
    edgeMin   = 10
    edgeMax   = 55
    if (x1 < edgeMin) or (x1 > edgeMax):
      oneVertex[0] = 1

    if (y1 < edgeMin) or (y1 > edgeMax):
      oneVertex[1] = 1

    if (x2 < edgeMin) or (x2 > edgeMax):
      twoVertex[0] = 1

    if (y2 < edgeMin) or (y2 > edgeMax):
      twoVertex[1] = 1

    if ( sum(oneVertex) > 0 ) and ( sum(twoVertex) > 0 ):
      print "%%%"
      print x1, y1
      print x2, y2 
      print "Line Detected"
      return True
      #if ( sum(oneVertex) + sum(twoVertex) ) == 4:
      #  print "Line is an edge"
        # return False
      # else:
      # return True 
    else:
      print "%%%"
      print x1, y1
      print x2, y2 
      print "Not Detected"
      return False
  
  def callback(self,data):
    try:
      # read image from topic
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

      # do processing to detect edges
      cv_gray = cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY)
      cv_edges = cv2.Canny(cv_gray, self.cThresh1, self.cThresh2, 3) #apertureSize = self.cApertureSize)
      cv_lines = cv2.HoughLinesP(cv_edges, self.rho, self.theta, self.houghThresh, minLineLength = self.houghMinLen, maxLineGap = self.houghMaxGap)
      num_line = 0
      if cv_lines is not None:
        for x1,y1,x2,y2 in cv_lines[0]:
          if self.checkHoughs(x1,y1,x2,y2):
            num_line = num_line + 1 
            cv2.line(cv_image,(x1,y1),(x2,y2),(0,255,0),2)
      if num_line > 0:
        print "Line Detected" 
 

      # show image
      temp = cv_image.copy()
      temp /= 2
      temp[cv_edges != 0] = (255, 255, 255)
      h = cv_image.shape[0]
      w = cv_image.shape[1]
      cv_out = cv2.resize(temp, (int(w), int(h)))
  

      #cv_image = cv_edges
      #cv2.imshow('houghlines',cv_out)

    except CvBridgeError, e:
      cv2.destroyAllWindows()

    # (rows,cols,channels) = cv_image.shape
    # if cols > 60 and rows > 60 :
      # cv2.circle(cv_image, (50,50), 10, 255)

    # cv2.imshow("Image window", cv_edges)
    # cv2.waitKey(3)

    try:
      self.canny_pub.publish(self.bridge.cv2_to_imgmsg(cv_out, "bgr8"))    # to see Canny edges
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))  # to see Detected lines
      self.nline_pub.publish(num_line) # number of lines detected
    except CvBridgeError, e:
      print e

def main(args):
  ic = line_finder()
  #time.sleep(30)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
 
