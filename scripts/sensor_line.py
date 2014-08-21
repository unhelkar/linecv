#!/usr/bin/env python
import roslib
roslib.load_manifest('linecv')

import rospy
import time
import math
import numpy 

from std_msgs.msg import String, Float64, Float64MultiArray
from linecv.msg import nline_xy
from belt_sensing.msg import mergedOpticalFlow
from belt_sensing.msg import sensorState
from belt_sensing.msg import mergedSensorStates

class SensorOnLine():
	def __init__(self):
		
		self.pub = rospy.Publisher("sensor_line_XY", nline_xy)

		rospy.init_node('sensorLineNode')

		self.subFL = rospy.Subscriber("opticFL/nline", Float64, self.recordMeas, callback_args = 1)
		self.subFR = rospy.Subscriber("opticFR/nline", Float64, self.recordMeas, callback_args = 2)
		self.subBL = rospy.Subscriber("opticBL/nline", Float64, self.recordMeas, callback_args = 3)
		self.subBR = rospy.Subscriber("opticBR/nline", Float64, self.recordMeas, callback_args = 4)

		self.subLoc = rospy.Subscriber("/optic_near_belt",  Float64MultiArray, self.recordLocal)

		self.lineXY   = nline_xy()

		self.lineXY.time   = rospy.Time.now()
		self.lineXY.hough  = [0, 0, 0, 0]
		self.lineXY.lclzn  = [0, 0, 0, 0]
		self.lineXY.yhat   = [0, 0, 0, 0]

	def recordMeas(self, msg, callback_args):
		#sielf.lineXY   = nline_xy()
		self.lineXY.time = rospy.Time.now()

		if callback_args == 1:
			self.lineXY.hough[0] = msg.data
		if callback_args == 2:
			self.lineXY.hough[1] = msg.data
		if callback_args == 3:
			self.lineXY.hough[2] = msg.data
		if callback_args == 4:
			self.lineXY.hough[3] = msg.data

		#print str(self.lineXY.lclzn)
		for i in xrange(4):
			self.lineXY.yhat[i] = self.lineXY.hough[i]*self.lineXY.lclzn[i]
			#print str(i) #self.lineXY.yhat[i])


	def recordLocal(self, msg):
		self.lineXY.lclzn = msg.data 

	def run(self):
		while not rospy.is_shutdown():
			self.pub.publish(self.lineXY)
			time.sleep(0.01)

if __name__=="__main__":
		try:
			nlineXY_node = SensorOnLine()
			nlineXY_node.run()
		except	rospy.ROSInterruptException:
			pass
		
