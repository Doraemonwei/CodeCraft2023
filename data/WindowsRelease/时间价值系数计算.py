# -*- coding: utf-8 -*-
# @Time : 2023/3/20 11:36
# @Author : Lanpangzi
# @File : 时间价值系数计算.py
import math

max_x = 9000
min_rate = 0.8


def second2frame(s):
    return 50 * s


x = 10


def f(x, max_x=9000, min_rate=0.8):
    x = second2frame(x)
    first_1 = 1-math.sqrt(1-(1-x/max_x)**2)
    first_2 = 1-min_rate
    return first_1*first_2+min_rate


print(f(x=10))
