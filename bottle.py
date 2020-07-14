# coding=utf-8
from __future__ import print_function, division, generators, absolute_import

from array import array
import time
import select
import multiprocessing

import lcm
from communication.robot_control_t import robot_control_t
from communication.laser_t import laser_t
from communication.pose_t import pose_t
import pickle as pkl
import os
import sys

os.environ['LCM_DEFAULT_URL'] = 'udpm://224.0.0.1:7667?ttl=1'

CH_SEND = 'SEND_COMMAND'
CH_CHASSIS_INFO = 'TEST_COMMAND'
CH_LASER_DATA = 'LASER_DATA'
CH_POSE = 'POSE'
TIMEOUT = 1.5
TIME_PRECISION = 100000

CHASSIS_INFO = [0 for _ in range(19)]


class Controller:
    """
    底盘控制
    """

    def __init__(self):
        self.lc = lcm.LCM()
        self._heartbeat_process = None
        self._heartbeat_time = None
        self.lc.subscribe(CH_CHASSIS_INFO, chassis_info_handler)
        self.chassis_info = CHASSIS_INFO

    def __del__(self):
        self.stop_heartbeat()

    def start_heartbeat(self, interval=0.1):
        """
        开启心跳包发送进程
        :param interval: 发送心跳包的时间间隔
        :return:
        """
        # 发送心跳包的进程
        self._heartbeat_time = multiprocessing.Value("d", interval)  # 心跳时间(s) 进程内共享变量
        self._heartbeat_process = \
            multiprocessing.Process(target=self._heartbeat_daemon, args=(self._heartbeat_time,))
        self._heartbeat_process.start()

    def stop_heartbeat(self):
        """
        结束发心跳包
        """
        if self._heartbeat_process:
            print('stop')
            self._heartbeat_time.value = 0
            self._heartbeat_process.join()
            self._heartbeat_process = None

    def _heartbeat_daemon(self, heartbeat_time):
        while heartbeat_time.value > 0:
            self.heartbeat(heartbeat_time.value * 30)
            time.sleep(heartbeat_time.value)

    def heartbeat(self, time):
        """
        发送心跳信号
        :param time:
        :return:
        """
        msg = self._build_control_msg(
            commandid=4,
            dparams=[time]
        )
        self.lc.publish(CH_SEND, msg.encode())

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

    def move(self, velocity=0.0, direction=0.0):
        """
        按照给定速度移动 (默认参数为全0 即停止)
        :param velocity: 速度(-0.9~0.9)
        :param direction: 舵角(-31~31)
        :return:
        """
        msg = self._build_control_msg(
            commandid=1,
            dparams=[velocity, direction]
        )
        self.lc.publish(CH_SEND, msg.encode())

    def query_messages(self):
        """
        query messages from chassis
        :return:
        """
        self.lc.handle()
        self.chassis_info = CHASSIS_INFO

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
    global CHASSIS_INFO
    msg = robot_control_t.decode(data)
    if msg.commandid == 4:
        print('底盘信息: ', msg.iparams)
        CHASSIS_INFO = msg.iparams


def chassis_data_handler(channel, data):
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
    # controller.start_heartbeat()
    controller.protection(False, False, True)
    # controller.heartbeat(time=3)
    if len(sys.argv) > 1:
        controller.light(int(sys.argv[1]))

    # controller.move(-0.1, 0)
    # lc = lcm.LCM()
    # subscriptions = [
    #     # lc.subscribe(CH_CHASSIS_INFO, chassis_info_handler),  # 底盘通信
    #     # lc.subscribe(CH_LASER_DATA, laser_info_handler),  # 激光通信
    #     # lc.subscribe(CH_POSE, pose_info_handler),  # 姿态
    #
    # ]
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
