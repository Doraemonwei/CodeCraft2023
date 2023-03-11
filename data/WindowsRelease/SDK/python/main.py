#!/bin/bash
import math
import sys
from collections import defaultdict

from load_map import read_map_util_ok

# 全局记录机器人的状态，''表示没有任务, 其他的就跟上面一样表示要去的工作台id和达到之后需要进行的操作
robots_status = ['', '', '', '']


def finish():
    sys.stdout.write('OK\n')
    sys.stdout.flush()


# 读取每一帧的状态
def read_status():
    s_input = input()
    row = 1  # 上来就直接是1，第一个已经
    K = 0  # 工作台数量
    m_benches = []  # 这一帧传进来的各个工作台的状态
    m_robots = []  # 这一帧传进来的各个机器人的状态
    while s_input != "OK":
        if row == 1:
            input_line = s_input.split(' ')
            K = int(input_line[0])

        elif row == 2:
            bench_id = 0  # 记录工作台的唯一id,其实就是在m_benched中的索引
            t = s_input.split(' ')
            # [工作台id, 工作台类型，[x,y], 剩余生产时间（帧）， 原材料状态（转成二进制使用）， 产品格状态]
            m_benches.append([bench_id, int(t[0]), [float(t[1]), float(t[2])], int(t[3]), int(t[4])])
            for j in range(K - 1):
                bench_id += 1
                t = input().split(' ')
                m_benches.append([bench_id, int(t[0]), [float(t[1]), float(t[2])], int(t[3]), int(t[4])])
        else:
            t = s_input.split(' ')
            # [可以交易的工作台id，携带物品类型，时间价值系数，碰撞价值系数, 角速度，线速度[x,y]，朝向，坐标[x,y]]
            m_robots.append([int(t[0]), int(t[1]), float(t[2]), float(t[3]), float(t[4]), [float(t[5]), float(t[6])],
                             float(t[7]), [float(t[8]), float(t[9])]])
            for i in range(3):
                t = input().split(' ')
                m_robots.append(
                    [int(t[0]), int(t[1]), float(t[2]), float(t[3]), float(t[4]), [float(t[5]), float(t[6])],
                     float(t[7]), [float(t[8]), float(t[9])]])
        s_input = input()
        row += 1
    return m_benches, m_robots


# 计算这一帧所有这个机器人应该执行的指令
#    机器人当前的坐标，机器人当前的角度,目标工作台的坐标
#    输出的是这一帧的这个机器人线速度和角度
def cal_instruct(robot_loc, robot_angle, bench_loc):
    r_x, r_y = robot_loc[0], robot_loc[1]
    b_x, b_y = bench_loc[0], bench_loc[1]

    r2b = [b_x - r_x, b_y - r_y]  # 机器人指向工作台的向量，目标就是把机器人的速度矢量掰过来
    r2b_a = math.atan2(r2b[1], r2b[0])  # 当前机器人与目标工作台向量与x轴正方向的夹角

    distance = math.sqrt((r_x - b_x) ** 2 + (r_y - b_y) ** 2)  # 当前机器人与工作台的距离

    n_line_speed = 2
    n_angle_speed = 0
    if r2b_a >= 0 and robot_angle >= 0:
        if robot_angle > r2b_a:
            n_angle_speed = -3
        elif robot_angle < r2b_a:
            n_angle_speed = 3
    elif r2b_a < 0 and robot_angle < 0:
        if robot_angle > r2b_a:
            n_angle_speed = -3
        elif robot_angle < r2b_a:
            n_angle_speed = 3
    elif r2b_a < 0 and robot_angle > 0:
        if abs(r2b_a) + abs(robot_angle) <= math.pi:
            n_angle_speed = -3
        else:
            n_angle_speed = 3
    else:
        if abs(r2b_a) + abs(robot_angle) <= math.pi:
            n_angle_speed = 3
        else:
            n_angle_speed = -3

    if distance >= 6:
        n_line_speed = 6
    return [n_line_speed, n_angle_speed]


# 帮助函数，传进来工作台类型和原材料格子状态，返回差的材料类型
each_bench_materials = {4: {1, 2}, 5: {1, 3}, 6: {2, 3}, 7: {4, 5, 6}, 8: {7}, 9: {1, 2, 3, 4, 5, 6, 7}}  # 方便判断缺不缺材料


def lack_which_material(bench_type, material_status):
    had_material = set()
    for i in range(10):
        if (material_status >> i) & 1 == 1:
            had_material.add(i)
    lack_material = each_bench_materials[bench_type] - had_material
    return list(lack_material)


# 对于每一帧，初始化四个参数，分别记录：
# 1、type_lack,全场每一种物品缺多少,0就是不缺
# 2、robot_carry, 每一个机器人身上都携带着什么类型的物品，没有就是-1
# 3、each_lack, 对于缺的物品种类，按照种类分成若干份，记录是哪个工作台缺的，以及这个工作台的坐标
# 4、done_bench, 对于已经有成品的工作台，按照成品的种类将这些工作台分类，记录工作台id和坐标
# 5、each_lack_num, 每一种材料缺的数量-有机器人正在运输的数量，得到每一种材料真正缺少的数量
def init_frame():
    type_lack = [0] * 8  # 一共其中物品，分别用索引表示, 索引0并不存在，永远是0
    each_lack = defaultdict(lambda: [])
    done_bench = defaultdict(lambda: [])
    for bench in n_benches:
        # 工作台123不需要原材料, 但是可能有成品
        if bench[1] in [1, 2, 3]:
            if bench[5] == 1:
                # 对于工作台123，工作台类型就是产品的类型
                done_bench[bench[1]].append([bench[0], bench[2]])
            continue
        for lack_m in lack_which_material(bench[1], bench[3]):
            type_lack[lack_m] += 1
            each_lack[lack_m].append([bench[0], bench[2]])  # 注意bench[2]本来就是个列表[x,y]
        # 循环能到这里说明不是123，89又没有产品
        if bench[1] not in [8, 9]:
            if bench[5] == 1:
                done_bench[bench[1]].append([bench[0], bench[2]])
    robot_carry = [0] * 10  #  9种材料，哪些被机器人带着的

    each_lack_num = [0]*8 # 1-7，  7种材料分别实际还缺多少

    # 初始化就是不管当前正在运输的材料的数量，只看缺的
    for lack_type, lack_num in each_lack.items():
        each_lack_num[lack_type] = len(lack_num)

    # 记录机器人一共携带了多少个什么类型的材料
    for ind, robot in enumerate(n_robots):
        robot_carry[robot[1]] += 1

    # 将缺少的材料的个数-已经正在运输中的这个材料的个数
    for ind, lack_num in enumerate(robot_carry):
        if lack_num!=0:
            each_lack_num[ind] -= lack_num

    return type_lack, robot_carry, each_lack, done_bench, each_lack_num


def task_process():
    # 分别对每一个机器人进行操作，机器人id是0123
    # 首先是0号机器人
    #    如果0号机器人没有携带
    if n_robots[0][1]==0:






if __name__ == '__main__':

    # 读取机器人以及工作台的初始位置信息
    robots, benches = read_map_util_ok()
    # 每一次与判题器交互后都要输出ok并flush
    finish()

    while True:
        # 第一个必须由外面的循环读取，否则不能判断是否已经结束
        line = sys.stdin.readline()
        if not line:
            break
        parts = line.split(' ')
        frame_id = int(parts[0])
        # 读取信息并得到当前工作台和机器人的状态信息
        n_benches, n_robots = read_status()
        # 处理好每一帧需要的4个数据
        n_type_lack, n_robot_carry, n_each_lack, n_done_bench, n_each_lack_num = init_frame()

        sys.stdout.write('%d\n' % frame_id)
        line_speed, angle_speed = 3, 1.5
        for r_id in range(4):
            sys.stdout.write('forward %d %d\n' % (r_id, line_speed))
            sys.stdout.write('rotate %d %f\n' % (r_id, angle_speed))
        finish()
