#! /usr/bin/env python
# -*- coding:utf-8 -*-
import rospy
import numpy as np
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
from math import pi

# v = 10  # Velocidade linear
# w = 5  # Velocidade angular

# if __name__ == "__main__":
#     rospy.init_node("roda_exemplo")
#     pub = rospy.Publisher("cmd_vel", Twist, queue_size=3)

#     try:
#         while not rospy.is_shutdown():
#             vel = Twist(Vector3(v,0,0), Vector3(0,0,w))
#             pub.publish(vel)
#             rospy.sleep(2.0)
#     except rospy.ROSInterruptException:
#         print("Ocorreu uma exceção com o rospy")

def scaneou(dado):
	print("Faixa valida: ", dado.range_min , " - ", dado.range_max )
	print("Leituras:")
	print(np.array(dado.ranges).round(decimals=2))
	#print("Intensities")
	#print(np.array(dado.intensities).round(decimals=2))

	


if __name__=="__main__":

		rospy.init_node("roda2")

		velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 3 )


		while not rospy.is_shutdown():
			vel_frente = Twist(Vector3(0.15,0,0), Vector3(0,0,0))
			vel_parado = Twist(Vector3(0,0,0), Vector3(0,0,0))
			vel_curva = Twist(Vector3(0,0,0), Vector3(0,0, pi/10))

			velocidade_saida.publish(vel_parado)
			print('stop')
			rospy.sleep(2)

			velocidade_saida.publish(vel_frente)
			print('forward')
			rospy.sleep(5.2)
			
			velocidade_saida.publish(vel_parado)
			print('stop')
			rospy.sleep(2)
			
			velocidade_saida.publish(vel_curva)
			print('curvation')
			rospy.sleep(5)
	



