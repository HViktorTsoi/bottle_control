# coding=utf-8
from __future__ import print_function, division, generators, absolute_import

from array import array
import time
import select

import lcm
from communication.robot_control_t import robot_control_t
from communication.laser_t import laser_t
from communication.pose_t import pose_t
import pickle as pkl

CH_SEND = 'SEND_COMMAND'
CH_CHASSIS_INFO = 'TEST_COMMAND'
CH_LASER_DATA = 'LASER_DATA'
CH_POSE = 'POSE'
TIMEOUT = 1.5
TIME_PRECISION = 100000


class Controller:
    """
    底盘控制
    """

    def __init__(self):
        self.lc = lcm.LCM()

    def request_info(self):
        """
        请求底盘控制信息
        注意此函数没有回调 返回的底盘信息在返回流中获取
        :return:
        """
        msg = self._build_control_msg(
            commandid=3
        )
        self.lc.publish(CH_SEND, msg.encode())

    def protection(self, laser, sonar, collision):
        """
        保护措施开关
        :param laser: 雷达避停
        :param sonar: 声呐避停
        :param collision: 前后保险杠避停
        :return:
        """
        msg = self._build_control_msg(
            commandid=2,
            iparams=[int(laser), int(sonar), int(collision)]
        )
        self.lc.publish(CH_SEND, msg.encode())

    def light(self, bulb_id):
        """
        灯光信号
        :param bulb_id: 灯光id
        :return:
        """
        msg = self._build_control_msg(
            commandid=5,
            iparams=[bulb_id]
        )
        self.lc.publish(CH_SEND, msg.encode())

    def move(self, velocity=0, direction=0):
        """
        按照给定速度移动 (默认参数为全0 即停止)
        :param velocity: 速度(-0.9~0.9)
        :param direction: 舵角(-31~01)
        :return:
        """
        msg = self._build_control_msg(
            commandid=1,
            dparams=[velocity, direction]
        )
        self.lc.publish(CH_SEND, msg.encode())

    def _build_control_msg(self, commandid, sparams=[], bparams=''.encode(), dparams=[], iparams=[]):
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


def chassis_info_handler(channel, data):
    msg = robot_control_t.decode(data)
    # 判断不同的数据
    if msg.commandid == 2:
        print('速度: ', msg.dparams)
    elif msg.commandid == 3:
        print('激光状态: ', msg.iparams)
    elif msg.commandid == 4:
        print('底盘信息: ', msg.iparams)
    elif msg.commandid == 5:
        print('错误信息: ', msg.iparams, msg.bparams.encode())
    elif msg.commandid == 6:
        print('温湿度: ', msg.dparams)
    elif msg.commandid in [7, 8]:
        print('CAN状态: ', msg.commandid, msg.iparams)
    elif msg.commandid == 11:
        print('声呐: ', msg.iparams)
    elif msg.commandid == 12:
        print('IMU: ', msg.dparams)
        print(time.time())
    else:
        print('MISC: ', msg.commandid, msg.iparams, msg.dparams, msg.bparams.encode())


def get_f32arr_timestamp():
    """
    获取浮点列表时间戳
    :return:
    """
    cur_time = time.time() * 1000
    first, second = cur_time // TIME_PRECISION, cur_time % TIME_PRECISION
    return [first, second]


def laser_info_handler(channel, data):
    tic = time.time()
    msg = laser_t.decode(data)
    print(msg.nranges, msg.nintensities, msg.ranges[400:410])
    with open('/tmp/laser.bin', 'ab') as file:
        # 保存距离 反射率 弧度 时间戳信息
        arr = array(
            'f', list(msg.ranges) + list(msg.intensities) + [msg.rad0, msg.radstep] + get_f32arr_timestamp())
        arr.tofile(file)
    toc = time.time()
    print('ellipsed', toc - tic)


def pose_info_handler(channel, data):
    msg = pose_t.decode(data)
    print('XYD: {}; ACC:{}; ORE:{}; ROT:{}'.format(msg.pos, msg.accel, msg.orientation, msg.rotation_rate))


if __name__ == '__main__':
    controller = Controller()
    controller.light(2)
    # lc = lcm.LCM()
    # subscriptions = [
    #     # lc.subscribe(CH_CHASSIS_INFO, chassis_info_handler),  # 底盘通信
    #     # lc.subscribe(CH_LASER_DATA, laser_info_handler),  # 激光通信
    #     # lc.subscribe(CH_POSE, pose_info_handler),  # 姿态
    #
    #]
    # try:
    #     while True:
    #         rfds, wfds, efds = select.select([lc.fileno()], [], [], TIMEOUT)
    #         if rfds:
    #             lc.handle()
    #         else:
    #             print('Waiting for messages..')
    # except Exception as e:
    #     print(e)
    #
    # # 注销订阅
    # for subscription in subscriptions: lc.unsubscribe(subscription)

    # request_info()
    # move(0, 0)
    # light(1)
    # time.sleep(1)
    # light(2)
    # protection(laser=False, sonar=False, collision=True)
