# -*- coding: utf-8 -*-
# @Time : 2023/3/10 13:04
# @Author : Lanpangzi
# @File : test.py
# 这个是测试文件，用来判断自己程序的某些小问题

import math
import platform
import time

print('系统:', platform.system())
T1 = time.perf_counter()

l = []
for i in range(100):
    l.append(0)
    l.pop()

T2 = time.perf_counter()
print('程序运行时间:%s毫秒' % ((T2 - T1) * 1000))
# 程序运行时间:0.0毫秒
