#!/usr/bin/python
# coding=utf-8
import time

import pygame
import pygame.locals
from pygame.locals import *
from bottle import Controller
import pygame.time

FPS = 10


class Chassis(pygame.sprite.Sprite):
    # 速度航向角范围
    MAX_SPEED = 0.9
    MIN_SPEED = -0.9
    MAX_LEADING = 31
    MIN_LEADING = -31

    # 调整速度的帧数间隔
    VELOCITY_CHANGE_INTERVAL = 3

    # 速度 航向角阈值变化
    DELTA_SPEED = 0.04
    DELTA_LEADING = 3

    def __init__(self):
        pygame.sprite.Sprite.__init__(self)
        self.velocity = 0
        self.leading = 0
        self.state = "still"
        self.velocity_change_count = 0

        self.reinit()

        # 控制包
        self.controller = Controller()
        self.controller.protection(False, False, True)

    def send_control(self):
        self.controller.move(velocity=self.velocity, direction=self.leading)
        print('{} V:{} W:{}'.format(self.state, self.velocity, self.leading))

    def reinit(self):
        self.velocity = 0
        self.leading = 0
        self.state = "still"

    def speed_up(self):
        self.state = "speed_up"
        # 等待一段时间再加速
        # 加速
        if self.velocity < self.MAX_SPEED:
            self.velocity += self.DELTA_SPEED
            self.send_control()

    def speed_down(self):
        # 减速
        self.state = "speed_down"
        if self.velocity > self.MIN_SPEED:
            self.velocity -= self.DELTA_SPEED
            self.send_control()

    def lead_left(self):
        # 左打舵
        self.state = "lead_left"
        if self.leading > self.MIN_LEADING:
            self.leading -= self.DELTA_LEADING
            self.send_control()

    def lead_right(self):
        # 右打舵
        if self.leading < self.MAX_LEADING:
            self.leading += self.DELTA_LEADING
            self.state = "lead_right"
            self.send_control()

    def lead_forward(self):
        if self.leading != 0:
            # 回轮
            self.leading = 0
            self.state = "still"
            self.send_control()

    def brake(self):
        if self.velocity != 0:
            self.velocity = 0
            self.state = "still"
            self.send_control()

    def stop(self):
        self.velocity = self.leading = 0
        self.state = "stop"
        self.send_control()

    def handle_key_action(self, key_pressed):
        """
        处理键盘对应动作
        :param key_pressed:
        :return:
        """
        up, down, left, right, space = key_pressed[pygame.K_UP], \
                                       key_pressed[pygame.K_DOWN], \
                                       key_pressed[pygame.K_LEFT], \
                                       key_pressed[pygame.K_RIGHT], \
                                       key_pressed[pygame.K_SPACE]
        # 键盘驱动
        if space:
            # 停止键优先级最高
            self.stop()
        else:
            # 动力键没有触发 则停止对应的动作
            if not (up or down):
                self.brake()
            if not (left or right):
                self.lead_forward()

            # 处理对应的指令
            # 速度要间隔处理
            if self.velocity_change_count > self.VELOCITY_CHANGE_INTERVAL:
                # 上下互斥
                if up:
                    self.speed_up()
                elif down:
                    self.speed_down()
                self.velocity_change_count = 0
            self.velocity_change_count += 1

            # 左右互斥
            if left:
                self.lead_left()
            elif right:
                self.lead_right()


def render_label(text, screen):
    # 显示文字
    font = pygame.font.Font(None, 36)
    text = font.render(text, 1, (10, 10, 10))
    textpos = text.get_rect()
    textpos.centerx = screen.get_rect().centerx
    textpos.centery = screen.get_rect().centery

    # 附着到屏幕
    screen.blit(text, textpos)


def main():
    # Initialise screen
    pygame.init()
    screen = pygame.display.set_mode((550, 250))
    pygame.display.set_caption('Chassis Control')

    pygame.display.flip()

    clock = pygame.time.Clock()

    chassis = Chassis()
    # Event loop
    while True:

        # 限制帧数
        clock.tick(FPS)

        # 处理键盘事件
        chassis.handle_key_action(pygame.key.get_pressed())

        for event in pygame.event.get():
            if event.type == pygame.locals.QUIT:
                return

        # 渲染新的画面
        screen.fill(pygame.Color('white'))
        render_label('{} V:{:.02f} W:{:.02f}'.format(chassis.state, chassis.velocity, chassis.leading), screen)
        pygame.display.flip()


if __name__ == '__main__': main()
