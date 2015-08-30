#!/usr/bin/python
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Point
import rospy
from numpy.linalg import svd
from scipy.optimize import leastsq

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import serial, time
import pickle

import tfx

class PointPub:
	def __init__(self):
		print('hey')

	def pub(self):
		rospy.init_node('point_pub', anonymous=True)
		self.publisher= rospy.Publisher("/chessboard_center", PoseStamped)
		while not rospy.is_shutdown():
			pose = tfx.pose([0.03776, 0.02868, 0.28153]).msg.PoseStamped()
			pose.header.frame_id = 'left_BC'
			self.publisher.publish(pose)
		rospy.spin()

if __name__ == '__main__':
	p = PointPub()
	p.pub()