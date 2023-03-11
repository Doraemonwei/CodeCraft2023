# -*- coding: utf-8 -*-
# @Time : 2023/3/10 12:48
# @Author : Lanpangzi
# @File : load_map.py
import sys

bench_ids = [str(i) for i in range(1, 10)]


# 初始化地图，得到所有工作台和机器人的坐标以及序号
def read_map_util_ok():
    robots = []
    benches = []
    row = 1
    m_input = input()
    robots_id = 0
    bench_id = 0
    while m_input != "OK":
        for ind, j in enumerate(m_input):
            if j == '.':
                continue
            elif j in bench_ids:
                # [唯一id,种类编号,[x,y]] 坐标是中心坐标
                benches.append([bench_id, int(j), [0.25 + (ind + 1) * 0.5, 50 - (0.25 + 0.5 * row)]])
                bench_id += 1
            else:
                # [唯一id, [x,y]] 坐标是中心坐标
                robots.append([robots_id, [0.25 + (ind + 1) * 0.5, 50 - (0.25 + 0.5 * row)]])
                robots_id += 1
    return robots, benches
