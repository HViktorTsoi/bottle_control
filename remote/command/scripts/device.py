#!/usr/bin/python
# coding=utf-8

import rospy
import pika
import thread
import pygame
import pygame.locals
from pygame.locals import *
from command.msg import chasis
import numpy as np

ZMQ_SERVER = "192.168.120.75"

frame = chasis()
chasis_pub = rospy.Publisher(name="/to_chasis", data_class=chasis, queue_size=1)

def callback(ch, method, properties, body):
	data = body.decode('utf-8')
	substr = data.split('/')
	speed = float(substr[0])
	heading = float(substr[1])
	frame.speed = speed
	frame.heading = heading
	chasis_pub.publish(frame)
	

if __name__ == '__main__' :

	rospy.init_node('command', anonymous=True)
	connection = pika.BlockingConnection(pika.ConnectionParameters(host=ZMQ_SERVER))
	channel = connection.channel()
	channel.queue_declare(queue='DeviceRec')
	channel.basic_consume(queue='DeviceRec', on_message_callback=callback, auto_ack=True)
	channel.start_consuming()
	connection.close()





