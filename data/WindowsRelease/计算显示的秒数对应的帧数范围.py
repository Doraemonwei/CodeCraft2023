# -*- coding: utf-8 -*-
# @Time : 2023/3/21 20:35
# @Author : Lanpangzi
# @File : 计算显示的秒数对应的帧数范围.py

def s2f(s):
    z = 50 * (180 - s)
    return [z, z + 50]


s = 136
print(s2f(s))
