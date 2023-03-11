# -*- coding: utf-8 -*-
# @Time : 2023/3/10 13:04
# @Author : Lanpangzi
# @File : test.py
# 这个是测试文件，用来判断自己程序的某些小问题

import platform
import time
from collections import defaultdict

print('系统:', platform.system())
T1 = time.perf_counter()

each_bench_materials = {4: {1, 2}, 5: {1, 3}, 6: {2, 3}, 7: {4, 5, 6}, 8: {7}, 9: {1, 2, 3, 4, 5, 6, 7}}  # 方便判断缺不缺材料


def lack_which_material(bench_type, material_status):
    had_material = set()
    for i in range(10):
        if (material_status >> i) & 1 == 1:
            had_material.add(i)
    lack_material = each_bench_materials[bench_type] - had_material
    return list(lack_material)

print(lack_which_material(4,0))

lac = {}


T2 = time.perf_counter()
print('程序运行时间:%s毫秒' % ((T2 - T1) * 1000))
# 程序运行时间:0.0毫秒
