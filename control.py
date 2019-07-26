#!/usr/bin/python
# coding=utf-8

import pygame
import pygame.locals
from pygame.locals import *


class Chassis(pygame.sprite.Sprite):
    # 速度航向角范围
    MAX_SPEED = 0.9
    MIN_SPEED = -0.9
    MAX_LEADING = 31
    MIN_LEADING = -31

    # 速度 航向角阈值变化
    DELTA_SPEED = 0.05
    DELTA_LEADING = 3

    def __init__(self):
        pygame.sprite.Sprite.__init__(self)
        self.velocity = 0
        self.leading = 0
        self.state = "still"
        self.reinit()

    def render(self):
        print('{} V:{} W:{}'.format(self.state, self.velocity, self.leading))

    def reinit(self):
        self.state = "still"
        self.velocity = 0
        self.leading = 0
        self.state = "still"

    def speed_up(self):
        # 加速
        self.velocity += self.DELTA_SPEED
        self.velocity = min(self.velocity, self.MAX_SPEED)
        self.state = "speed_up"
        self.render()

    def speed_down(self):
        # 减速
        self.velocity -= self.DELTA_SPEED
        self.velocity = max(self.velocity, self.MIN_SPEED)
        self.state = "speed_down"
        self.render()

    def lead_left(self):
        # 左打舵
        self.leading -= self.DELTA_LEADING
        self.leading = max(self.leading, self.MIN_LEADING)
        self.state = "lead_left"
        self.render()

    def lead_right(self):
        # 右打舵
        self.leading += self.DELTA_LEADING
        self.leading = min(self.leading, self.MAX_LEADING)
        self.state = "lead_right"
        self.render()

    def lead_forward(self):
        # 回轮
        self.leading = 0
        self.state = "forward"
        self.render()

    def brake(self):
        self.velocity = 0
        self.state = "still"
        self.render()

    def stop(self):
        self.velocity = self.leading = 0
        self.state = "still"
        self.render()


def main():
    # Initialise screen
    pygame.init()
    screen = pygame.display.set_mode((550, 250))
    pygame.display.set_caption('Chassis Control')

    # Fill background
    background = pygame.Surface(screen.get_size())
    background = background.convert()
    background.fill((250, 250, 250))

    # Display some text
    font = pygame.font.Font(None, 36)
    text = font.render("Chassis Control", 1, (10, 10, 10))
    textpos = text.get_rect()
    textpos.centerx = background.get_rect().centerx
    background.blit(text, textpos)

    # Blit everything to the screen
    screen.blit(background, (0, 0))
    pygame.display.flip()

    player = Chassis()
    # Event loop
    pygame.key.set_repeat(1, 100)
    while True:
        for event in pygame.event.get():
            if event.type == pygame.locals.QUIT:
                return
            # 键盘驱动
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    player.stop()
                if event.key == pygame.K_UP:
                    player.speed_up()
                if event.key == pygame.K_DOWN:
                    player.speed_down()
                if event.key == pygame.K_LEFT:
                    player.lead_left()
                if event.key == pygame.K_RIGHT:
                    player.lead_right()
            # 抬起键盘都复位
            elif event.type == pygame.locals.KEYUP:
                if event.key == pygame.K_UP or event.key == pygame.K_DOWN:
                    player.brake()
                if event.key == pygame.K_LEFT or event.key == pygame.K_RIGHT:
                    player.lead_forward()

        screen.blit(background, (0, 0))
        pygame.display.flip()


if __name__ == '__main__': main()
