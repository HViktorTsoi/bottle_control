# coding=utf-8
from __future__ import print_function, division, generators, absolute_import

import time

import lcm
from communication.robot_control_t import robot_control_t


def protection(laser, sonar, collision):
    """
    保护措施开关
    :param laser: 雷达避停
    :param sonar: 声呐避停
    :param collision: 前后保险杠避停
    :return:
    """
    msg = build_control_msg(
        commandid=2,
        iparams=[int(laser), int(sonar), int(collision)]
    )
    lc.publish("SEND_COMMAND", msg.encode())


def light(bulb_id):
    """
    灯光信号
    :param bulb_id: 灯光id
    :return:
    """
    msg = build_control_msg(
        commandid=5,
        iparams=[bulb_id]
    )
    lc.publish("SEND_COMMAND", msg.encode())


def move(velocity=0, direction=0):
    """
    按照给定速度移动 (默认参数为全0 即停止)
    :param velocity: 速度(-0.9~0.9)
    :param direction: 舵角(-31~01)
    :return:
    """
    msg = build_control_msg(
        commandid=1,
        dparams=[velocity, direction]
    )
    lc.publish("SEND_COMMAND", msg.encode())


def build_control_msg(commandid, sparams=[], bparams=''.encode(), dparams=[], iparams=[]):
    msg = robot_control_t()
    msg.commandid = commandid

    msg.nsparams = len(sparams)
    msg.sparams = sparams

    msg.nbparams = len(bparams)
    msg.bparams = bparams

    msg.ndparams = len(dparams)
    msg.dparams = dparams

    msg.niparams = len(iparams)
    msg.iparams = iparams

    return msg


if __name__ == '__main__':
    lc = lcm.LCM()
    move(0, 0)
    # light(1)
    # time.sleep(1)
    # light(2)
    # protection(laser=False, sonar=False, collision=True)
