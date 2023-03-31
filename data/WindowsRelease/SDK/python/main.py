#!/bin/bash
import sys

from map_1_procedure import map_1_main
from map_2_procedure import map_2_main
from map_3_procedure import map_3_main
from map_4_procedure import map_4_main


# 初始化地图，得到所有工作台和机器人的坐标以及序号
def read_map_util_ok():
    s = ''
    cell_y = 99
    obs = set()
    t_input = input()
    while t_input != "OK":
        for ind, i in enumerate(t_input):
            if i != '.' and i != '#':
                s += i
            if i == '#':
                # if (ind, column) == (24, 60):
                #     test_write_file('错了你妈的')
                obs.add((ind, cell_y))
        t_input = input()
        cell_y -= 1
    return s, obs


def test_write_file(a):
    with open("test_file.txt", "a", encoding='utf-8') as variable_name:
        variable_name.write(str(a) + '\n')


# 给定当前是第几帧，判断现在应该显示几分几秒
def cal_time(frame):
    second, frame = divmod(frame, 50)  # 返回已经进行的秒数和余下来的帧数
    last_second = 180 - second
    last_min, last_second = divmod(last_second, 60)
    return str(last_min) + ' : ' + str(last_second)


def finish():
    sys.stdout.write('OK\n')
    sys.stdout.flush()


# 记录用的是哪一张地图，方便使用对应的分配方案和运动控制
# 地图1： 124235316AA25A431A78712543612423531
# 地图2：123132132AAAA56465478
# 地图3：71233AAAA4564568112233
# 地图4：912312544A5545A45666666455563652415546565A64A566661236
which_map = {'5112342AA33564478A9987883123A5446': 1, '62543A1AAA78': 2, '121AA324A37685A99': 3,
             '65A4A122287144124222218AA63355': 4}

if __name__ == '__main__':
    # 读取机器人以及工作台的初始位置信息
    map_mark, obs = read_map_util_ok()
    finish()
    # 每一次与判题器交互后都要输出ok并flush
    if which_map[map_mark] == 1:
        map_1_main(obs)
    elif which_map[map_mark] == 2:
        map_2_main()
    elif which_map[map_mark] == 3:
        map_3_main()
    else:
        map_4_main()
