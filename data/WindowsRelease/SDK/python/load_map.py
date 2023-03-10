# -*- coding: utf-8 -*-
# @Time : 2023/3/10 12:48
# @Author : Lanpangzi
# @File : load_map.py


bench_ids = [str(i) for i in range(1, 10)]


# 初始化地图，得到所有工作台和机器人的坐标以及序号
def read_map_util_ok():
    robots = []
    benches = []
    row = 1
    m_input = input()
    while m_input != "OK":
        for ind, j in enumerate(m_input):
            if j == '.':
                continue
            elif j in bench_ids:
                # [编号, [x,y]] 坐标是中心坐标
                benches.append([int(j), [0.25 + (ind+1) * 0.5, 50 - (0.25 + 0.5 * row)]])
            else:
                robots
