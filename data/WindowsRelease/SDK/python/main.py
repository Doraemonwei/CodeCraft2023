#!/bin/bash
import math
import sys

from load_map import read_map_util_ok

# 任务队列，将所有的任务都放在一个队列中，先进先出
#  任务队列格式 "010"+str(task_id),前两位是目标工作台id，第三位是操作，0表示去买，1表示去卖, task_id
#    用来唯一标记这个任务，判断这个任务有没有被执行
#  初始时没有任务，其实是有任务的，但是初始化地图时先不分配，等到开始交互了再分配
tasks = []

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
            # [工作台id，携带物品类型，时间价值系数，碰撞价值系数, 角速度，线速度[x,y]，朝向，坐标[x,y]]
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


# 检查是否有机器人已经达到目标工作台，如果有，执行买卖任务
#   遍历四个机器人的任务，如果有身上背着任务的而且已经达到目标工作台了，则执行任务并且将状态改成''
def do_task():
    # 返回的是这一帧机器人是否需要买卖，0是买，1是卖，-1是不操作
    re_sell_buy = [-1, -1, -1, -1]
    for ind, t in enumerate(robots_status):
        if t == '':
            continue
        # 检查这一帧判题器传过来的机器人状态，判断是否可以进行操作
        for rind, r in enumerate(n_robots):
            if r[0] == int(robots_status[ind][:2]):
                re_sell_buy[ind] = int(r[2])
                robots_status[ind] = ''
    return re_sell_buy


# 从任务队列中拿任务分配给机器人
def allocating_task():
    pass


# 生成任务并推进队列
#   这一块是将来优化的关键点
def generate_task():

    pass



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

        sys.stdout.write('%d\n' % frame_id)
        line_speed, angle_speed = 3, 1.5
        for r_id in range(4):
            sys.stdout.write('forward %d %d\n' % (r_id, line_speed))
            sys.stdout.write('rotate %d %f\n' % (r_id, angle_speed))
        finish()
