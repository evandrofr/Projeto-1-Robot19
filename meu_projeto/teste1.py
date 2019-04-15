#! /usr/bin/env python
# -- coding:utf-8 --


import rospy
import numpy as np
from math import pi
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
from std_msgs.msg import UInt8



bumper = None
def scaneou(data):
	global bumper
	bumper = data.data
	return bumper
#	print("Intensities")
#	print(np.array(dado.intensities).round(decimals=2))

	


if __name__=="__main__":

	rospy.init_node("teste1")

	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 3 )
	recebe_bumper = rospy.Subscriber("/bumper", UInt8, scaneou)


	while not rospy.is_shutdown():
		# bumper = None
		if bumper is None or bumper == 0:
			velocidade = Twist(Vector3(0.2,0,0), Vector3(0,0,0))
			velocidade_saida.publish(velocidade)
			rospy.sleep(0.1)

		elif bumper == 1:
			parado = Twist(Vector3(0,0,0),Vector3(0,0,0))
			velocidade_saida.publish(parado)
			rospy.sleep(2)
			velocidade_re = Twist(Vector3(-0.2,0,0), Vector3(0,0,0))
			velocidade_saida.publish(velocidade_re)
			rospy.sleep(2)

			velocidade_angular = Twist(Vector3(0,0,0), Vector3(0,0,-pi/10))
			velocidade_saida.publish(velocidade_angular)
			rospy.sleep(3)
			bumper = 0
			
		elif bumper == 2:
			parado = Twist(Vector3(0,0,0),Vector3(0,0,0))
			velocidade_saida.publish(parado)
			rospy.sleep(2)
			velocidade_re = Twist(Vector3(-0.2,0,0), Vector3(0,0,0))
			velocidade_saida.publish(velocidade_re)
			rospy.sleep(2)

			velocidade_angular = Twist(Vector3(0,0,0), Vector3(0,0,pi/10))
			velocidade_saida.publish(velocidade_angular)
			rospy.sleep(3)
			bumper = 0

		elif bumper == 3:
			parado = Twist(Vector3(0,0,0),Vector3(0,0,0))
			velocidade_saida.publish(parado)
			rospy.sleep(2)
			velocidade_saida.publish(velocidade)
			rospy.sleep(2)

			velocidade_angular = Twist(Vector3(0,0,0), Vector3(0,0,-pi/10))
			velocidade_saida.publish(velocidade_angular)
			rospy.sleep(3)
			bumper = 0	

		elif bumper == 4:
			parado = Twist(Vector3(0,0,0),Vector3(0,0,0))
			velocidade_saida.publish(parado)
			rospy.sleep(2)
			velocidade_saida.publish(velocidade)
			rospy.sleep(2)

			velocidade_angular = Twist(Vector3(0,0,0), Vector3(0,0,pi/10))
			velocidade_saida.publish(velocidade_angular)
			rospy.sleep(3)
			bumper = 0

			


			# velocidade = Twist(Vector3(0.05, 0, 0), Vector3(0, 0, 0))
			# velocidade_saida.publish(velocidade)
			# rospy.sleep(0.1)
			# if bumper == 1:
			# 	print(bumper)

			# if bumper == 2:
			# 	print(bumper)

			# if bumper == 3:
			# 	velocidade = Twist(Vector3(-0.1, 0, 0), Vector3(0, 0, 0))
			# 	velocidade_saida.publish(velocidade)
			# 	rospy.sleep(3)
			# 	print(bumper)

			# if bumper == 4:
			# 	print(bumper)

			# bumper = None
		
		print(bumper)
#		velocidade = Twist(Vector3(-0.0, 0, 0), Vector3(0, 0, 0))
#		velocidade_saida.publish(velocidade)
		# rospy.sleep(0.5)