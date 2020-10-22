#!/usr/bin/python
# coding=utf-8

import rospy
import pika
import thread
import pygame
import pygame.locals
from pygame.locals import *
from command.msg import chasis

ZMQ_SERVER = "47.97.42.104"

frame = chasis()

FPS = 10
MAX_SPEED = 0.9
MIN_SPEED = -0.9
MAX_HEADING = 31 #right
MIN_HEADING = -31 #left
DELTA_SPEED = 0.04
DELTA_HEADING = 6



def render_label(text, screen):
    font = pygame.font.Font(None, 36)
    text = font.render(text, 1, (10, 10, 10))
    textpos = text.get_rect()
    textpos.centerx = screen.get_rect().centerx
    textpos.centery = screen.get_rect().centery

    screen.blit(text, textpos)

if __name__ == '__main__' :

	pygame.init()
	screen = pygame.display.set_mode((1000, 600))
	pygame.display.set_caption('Remote Control')
	pygame.display.flip()
	clock = pygame.time.Clock()
	rospy.init_node('command', anonymous=True)
	chasis_pub = rospy.Publisher(name="/chasis", data_class=chasis, queue_size=1)
	connection = pika.BlockingConnection(pika.ConnectionParameters(host=ZMQ_SERVER))
	rate = rospy.Rate(10)
	channel = connection.channel()
	channel.queue_declare(queue='DeviceRec')
	while not rospy.is_shutdown():		
		clock.tick(FPS)
		keys_pressed = pygame.key.get_pressed()
		if keys_pressed[pygame.K_SPACE]:
			frame.speed = 0
			frame.heading = 0
		elif keys_pressed[pygame.K_UP]:
			if frame.speed >= MAX_SPEED:
				frame.speed = MAX_SPEED
			else:
				frame.speed += DELTA_SPEED
		elif keys_pressed[pygame.K_DOWN]:
			if frame.speed <= MIN_SPEED:
				frame.speed = MIN_SPEED
			else:
				frame.speed -= DELTA_SPEED
		elif keys_pressed[pygame.K_RIGHT]:
			if frame.heading >= MAX_HEADING:
				frame.heading = MAX_HEADING
			else:
				frame.heading += DELTA_HEADING
		elif keys_pressed[pygame.K_LEFT]:
			if frame.heading <= MIN_HEADING:
				frame.heading = MIN_HEADING
			else:
				frame.heading -= DELTA_HEADING
		else:
			pass
		pygame.event.pump()
		screen.fill(pygame.Color('white'))
		render_label('{}  V:{:.02f} H:{:.02f}'.format("Cloud Control:", frame.speed, frame.heading), screen)
		pygame.display.flip()
		data = str(frame.speed)+"/"+str(frame.heading)
		print(data)
		channel.basic_publish(exchange='', routing_key='DeviceRec', body=data)
		chasis_pub.publish(frame)
	connection.close()
