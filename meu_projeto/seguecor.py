#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import numpy as np
import tf
import math
import cv2
import time
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cormodule
from math import pi
from std_msgs.msg import UInt8


bridge = CvBridge()
cv_image = None
media = []
centro = []
dx = None
dy = None

def acha_cor(imagem):
	global cv_image
	global media
	global centro

	cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
	media, centro, area =  cormodule.identifica_cor(cv_image) #media -> centro do contorno #Centro -> centro da tela
	global dx
	dx = media[0] - centro[1] # centro (y,x) media (x,y)
	global dy
	dy = media[1] - centro[0]

bumper = None
def scaneou(data):
	global bumper
	bumper = data.data
	return bumper



if __name__=="__main__":

	rospy.init_node("seguecor")

	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 3 )
	recebe_cam = rospy.Subscriber("/kamera",CompressedImage,acha_cor)
	recebe_bumper = rospy.Subscriber("/bumper", UInt8, scaneou)

	velocidade_angular_p = Twist(Vector3(0,0,0), Vector3(0,0,pi/10))
	velocidade_ang_reto_p = Twist(Vector3(0.05,0,0), Vector3(0,0,pi/2))
	velocidade_angular_n = Twist(Vector3(0,0,0), Vector3(0,0,-pi/10))
	velocidade_ang_reto_n = Twist(Vector3(0.05,0,0), Vector3(0,0,-pi/2))
	velocidade_re = Twist(Vector3(-0.2,0,0), Vector3(0,0,0))
	velocidade = Twist(Vector3(0.2,0,0), Vector3(0,0,0))
	parado = Twist(Vector3(0,0,0),Vector3(0,0,0))


	dx_max = 30

	while not rospy.is_shutdown():

		if dx < dx_max:
			velocidade_saida.publish(velocidade_ang_reto_n)
		elif dx > -dx_max:
			velocidade_saida.publish(velocidade_ang_reto_p)



		if bumper is None or bumper == 0:
			
			velocidade_saida.publish(velocidade)
			rospy.sleep(0.1)

		elif bumper == 1:
			velocidade_saida.publish(parado)
			rospy.sleep(2)

			velocidade_saida.publish(velocidade_re)
			rospy.sleep(2)

			velocidade_saida.publish(velocidade_angular_n)
			rospy.sleep(3)
			bumper = 0
			
		elif bumper == 2:
			velocidade_saida.publish(parado)
			rospy.sleep(2)

			velocidade_saida.publish(velocidade_re)
			rospy.sleep(2)

			velocidade_saida.publish(velocidade_angular_p)
			rospy.sleep(3)
			bumper = 0

		elif bumper == 3:
			velocidade_saida.publish(parado)
			rospy.sleep(2)

			velocidade_saida.publish(velocidade)
			rospy.sleep(2)

			velocidade_saida.publish(velocidade_angular_n)
			rospy.sleep(3)
			bumper = 0	

		elif bumper == 4:
			velocidade_saida.publish(parado)
			rospy.sleep(2)
			velocidade_saida.publish(velocidade)
			rospy.sleep(2)

			velocidade_saida.publish(velocidade_angular_p)
			rospy.sleep(3)
			bumper = 0

