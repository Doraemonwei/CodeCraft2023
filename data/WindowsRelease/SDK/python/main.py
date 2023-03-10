#!/bin/bash
import sys
from load_map import read_map_util_ok


class Bench123:
    # 类型1、2、3的工作台
    def __init__(self, id_, pos, type_, product):
        self.id = id_  # 这个工作台的唯一id，标记用
        self.type = type_  # 这个bench是123中的哪一种
        self.product = product  # 生产出来的产品类型
        self.pos = pos
        self.run_period = 50  # 这个工作台的生产周期，其实123都是50
        self.left_time = 50  # 初始时就开始制造
        self.has_product = False  # 初始时没有产品

    # 拿走了已经生产好了的1号产品，改变剩余时间和是否含有产品的标记
    def take_product(self):
        self.left_time = 50
        self.has_product = False

    # 每一帧都需要对这个玩意进行更新
    def update(self):
        # 如果还需要时间生产，更新后剩余时间-1
        if self.left_time >= 1:
            self.left_time -= 1
        # 如果更新后发现生产完毕，跟新标记变量
        if self.left_time == 0:
            self.has_product = True


def finish():
    sys.stdout.write('OK\n')
    sys.stdout.flush()


if __name__ == '__main__':

    # 读取机器人以及工作台的位置信息
    robots, benches = read_map_util_ok()
    # 每一次与判题器交互后都要输出ok并flush
    finish()

    # 根据初始的map信息初始化所有的工作台和机器人信息

    while True:
        line = sys.stdin.readline()
        if not line:
            break
        parts = line.split(' ')
        frame_id = int(parts[0])
        read_util_ok()
        sys.stdout.write('%d\n' % frame_id)
        line_speed, angle_speed = 3, 1.5
        for robot_id in range(4):
            sys.stdout.write('forward %d %d\n' % (robot_id, line_speed))
            sys.stdout.write('rotate %d %f\n' % (robot_id, angle_speed))
        finish()
