#! /usr/bin/env python
# -- coding:utf-8 --


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
from sensor_msgs.msg import LaserScan
from std_msgs.msg import UInt8
from math import pi
import visao_module

bridge = CvBridge()

cv_image = None
media = []
centro_cor = []
centro_mobile = []
viu_bird = False

atraso = 1.5E9
area = 0.0
check_delay = False 
bumper = None
dist = []


def roda_todo_frame(imagem):
    global cv_image
    global media
    global centro_cor
    global centro_mobile

    global viu_bird

    now = rospy.get_rostime()
    imgtime = imagem.header.stamp
    lag = now-imgtime # calcula o lag
    delay = lag.nsecs
    if delay > atraso and check_delay==True:
        print("Descartando por causa do delay do frame:", delay)
        return 
    try:
        antes = time.clock()
        cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
        media, centro_cor, area =  cormodule.identifica_cor(cv_image)
        centro_mobile, imagem, resultados =  visao_module.processa(cv_image)
        depois = time.clock()


        for r in resultados:
            if r[0] == "bird":
                viu_bird = True




        cv2.imshow("Camera", cv_image)


    except CvBridgeError as e:
        print('ex', e)





def scaneou_bumper(data):
    global bumper
    bumper = data.data
    return bumper


def scaneou_scan(dado):
    global dist
    dist = (np.array(dado.ranges))
    for n in range(len(dist)):
        if dist[n] < dado.range_min:
            dist[n] = 1
    return dist

    


if __name__=="__main__":
    rospy.init_node("teste2")

    topico_imagem = "/kamera"



    recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)

    print("Usando ", topico_imagem)

    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

    recebe_bumper = rospy.Subscriber("/bumper", UInt8, scaneou_bumper)

    recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou_scan)





    velocidade_angular_p = Twist(Vector3(0,0,0), Vector3(0,0,-pi/10))
    velocidade_angular_n = Twist(Vector3(0,0,0), Vector3(0,0,pi/10))
    velocidade_re = Twist(Vector3(-0.2,0,0), Vector3(0,0,0))
    velocidade_fuga = Twist(Vector3(-0.3,0,0), Vector3(0,0,0.5))
    velocidade = Twist(Vector3(0.2,0,0), Vector3(0,0,0))
    parado = Twist(Vector3(0,0,0),Vector3(0,0,0))

    while not rospy.is_shutdown():

        vel = Twist(Vector3(0.15,0,0), Vector3(0,0,0))
        # print(media)

        if len(media) != 0 and len(centro_cor) != 0:
            # print("Média dos vermelhos: {0}, {1}".format(media[0], media[1]))
            # print("Centro dos vermelhos: {0}, {1}".format(centro_cor[0], centro_cor[1]))


            if media[0] - centro_cor[0] < -40:
                vel = Twist(Vector3(0.15,0,0), Vector3(0,0,0.5))
                print("Vermelho a Esquerda")

            elif media[0] - centro_cor[0] > 40:
                vel = Twist(Vector3(0.15,0,0), Vector3(0,0,-0.5))
                print("Vermelho a Direita")

            elif (media[0] - centro_cor[0] > -40) or (media[0] - centro_cor[0] < 40):
                vel = Twist(Vector3(0.15,0,0), Vector3(0,0,0))
                print("Vermelho Reto")


        if viu_bird:
            print("Bird! Fuga!")
            velocidade_saida.publish(parado)
            rospy.sleep(1)
            velocidade_saida.publish(velocidade_fuga)
            rospy.sleep(1)
            viu_bird = False

        # print("dist",dist)
        if dist != []:
            if ((min(dist[0:20]) < 0.25) or (min(dist[339:359]) < 0.25)):
                print("Quase batendo: Frente")
                # velocidade_saida.publish(parado)
                # rospy.sleep(1)
                velocidade_saida.publish(velocidade_fuga)
                rospy.sleep(1)
                velocidade_saida.publish(velocidade_angular_n)
                rospy.sleep(1)

            if (min(dist[79:99]) < 0.25):
                print("Quase batendo: Esquerda")
                desvio = Twist(Vector3(0,0,0), Vector3(0,0,-0.7))
                velocidade_saida.publish(desvio)
                rospy.sleep(1)


            if (min(dist[169:189]) < 0.25):
                print("Quase batendo: Costas")
                velocidade_saida.publish(velocidade)
                rospy.sleep(1)


            if (min(dist[259:279]) < 0.25):
                print("Quase batendo: Direita")
                desvio = Twist(Vector3(0,0,0), Vector3(0,0,0.7))
                velocidade_saida.publish(desvio)
                rospy.sleep(1)




        if bumper == 1:
            print("Bumper 1")
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
            
        elif bumper == 2:
            print("Bumper 2")
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

        elif bumper == 3:
            print("Bumper 3")
            parado = Twist(Vector3(0,0,0),Vector3(0,0,0))
            velocidade_saida.publish(parado)
            rospy.sleep(2)
            velocidade_saida.publish(velocidade)
            rospy.sleep(2)

            velocidade_angular = Twist(Vector3(0,0,0), Vector3(0,0,pi/10))
            velocidade_saida.publish(velocidade_angular)
            rospy.sleep(3)
            bumper = 0  

        elif bumper == 4:
            print("Bumper 4")
            parado = Twist(Vector3(0,0,0),Vector3(0,0,0))
            velocidade_saida.publish(parado)
            rospy.sleep(2)
            velocidade_saida.publish(velocidade)
            rospy.sleep(2)

            velocidade_angular = Twist(Vector3(0,0,0), Vector3(0,0,-pi/10))
            velocidade_saida.publish(velocidade_angular)
            rospy.sleep(3)
            bumper = 0



        velocidade_saida.publish(vel)
        rospy.sleep(0.1)

