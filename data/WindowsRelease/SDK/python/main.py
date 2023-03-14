#!/bin/bash
import math
import sys
import time
from collections import defaultdict

import numpy as np


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
            m_benches.append([bench_id, int(t[0]), [float(t[1]), float(t[2])], int(t[3]), int(t[4]), int(t[5])])
            for j in range(K - 1):
                bench_id += 1
                t = input().split(' ')
                m_benches.append([bench_id, int(t[0]), [float(t[1]), float(t[2])], int(t[3]), int(t[4]), int(t[5])])
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
    # test_write_file(m_benches)
    return m_benches, m_robots


# 设定机器人的基本参数
# robot parameter
# 线速度边界
v_max = 6.6  # m/s
v_min = 0  # m/s
# 角速度边界
w_max = 3  # rad/s
w_min = -3  # rad/s
# 线加速度和角加速度最大值
a_max_empty = 19 # m/s^2
a_wmax_empty = 38  # rad/s^2
a_vmax_carried = 14  # m/s^2
a_wmax_carried = 20 # rad/s^2

# 采样分辨率
v_sample = 0.5  # m/s
w_sample = 0.5  # rad/s
# 离散时间间隔,每一帧是50ms=0.05s
dt = 0.05  # s =50ms
# 轨迹推算时间长度,往后预测时间长度
predict_time = 0.6  # [s]
# 轨迹评价函数系数
alpha = 1.0  # 角度
beta = 1  # 距离
gamma = 1.0  # 速度
# 机器人半径 1m,实际上是不带货0.45，带货0.53，但是为了方便计算是否碰撞，直接设置为1m,这样边界就需要扩展0.5m了
robot_radius = 0.55  # m


# 机器人运动学模型，传入当前机器人状态信息，返回dt时间后的状态,dt就直接使用全局变量dt
#    state就是：[x(m), y(m), yaw(rad), v(m/s), w(rad/s)]
#    control就是对应的速度和角速度
def KinematicModel(state, control):
    """机器人运动学模型
    """
    # test_write_file('第一个state： ' + str(state))
    state[0] += control[0] * math.cos(state[2]) * dt
    state[1] += control[0] * math.sin(state[2]) * dt
    state[2] += control[1] * dt
    state[3] = control[0]
    state[4] = control[1]
    return state


# 计算符合要求的速度与角速度采样空间即 速度边界限制的+加速度限制+障碍物限制
# 传入的参数是是否携带了物品,其他三个机器人的位置信息,我自己的位置信息,以及这一瞬间的速度，角速度
def cal_dwv(is_carried, other_robot_loc, my_loc, v, w):
    # 速度边界限制的vm就是全局变量v_min,v_mix,w_min,w_max
    #
    # 加速度限制的速度范围，是否携带货物加速度是不一样的
    a_vmax = a_max_empty if not is_carried else a_vmax_carried
    a_wmax = a_wmax_empty if not is_carried else a_wmax_carried
    # 由最大加速度限制的速度区间,是在目前自己速度的基础上，参考最大加速度得到的
    v_low = v - a_vmax * dt
    v_high = v + a_vmax * dt
    w_low = w - a_wmax * dt
    w_high = w + a_wmax * dt

    # 障碍物限制的速度范围，障碍物只有其他三个机器人还有边界，这里暂时没有考虑边界
    #    首先找到到达距离我最近的障碍物的距离
    dis = float('inf')
    for robot_loc in other_robot_loc:
        dis = min(dis, math.sqrt((robot_loc[0] - my_loc[0]) ** 2 + (robot_loc[1] - my_loc[1]) ** 2))
    v_high_ob = math.sqrt(2 * dis * a_vmax)
    w_high_ob = math.sqrt(2 * dis * a_wmax)
    # test_write_file(
    #     "速度窗口各项参数v_min:{},v_max:{},v_low:{},v_high:{},v_high_ob:{}".format(v_min, v_max, v_low, v_high, v_high_ob))
    a = max(v_min, v_low)  # 机器人能达到的速度极限在目前速度条件下能通过减速到达的速度，这是最小的
    b = min(v_max, v_high, v_high_ob)  # 最大能到达的速度是由机器人自生极限，在目前速度状态下通过加速能到达的极限，被障碍限制能到达的极限
    c = max(w_min, w_low)
    d = min(w_max, w_high, w_high_ob)
    # test_write_file("速度窗口：{}".format([a, b, c, d]))
    return [a, b, c, d]


# 轨迹推算函数，传入当前时刻的机器人状态[x,y,yaw,v,w]和假设这个时刻给他的速度和角速度
def trajectory_predict(state_init, v, w):
    # test_write_file('传进来的初始化state： ' + str(state_init))
    trajectory = [state_init[:]]
    _time = 0
    # 在预测时间段内,这里是1秒
    while _time <= predict_time:
        # 注意这里是传入的最新一次路径的拷贝，因为KinematicModel函数会直接修改这个列表
        x = KinematicModel(trajectory[-1], [v, w])  # 运动学模型
        trajectory.append(x)
        _time += dt
    return trajectory


def eval_distance(trajectory, other_robot_loc):
    """
        距离评价函数，值越大越好
        返回这个轨迹在整个运动过程中距离障碍物的最小值，取整个过程中距离所距障碍物的最小值
    """
    r = float('inf')
    for tra in trajectory:
        for other_r in other_robot_loc:
            # 距离计算 根号下 (x1-x2)^2+(y1-y2)^2
            r = min(r, math.sqrt((tra[0] - other_r[0]) ** 2 + (tra[1] - other_r[1]) ** 2))
    return r


# 传进估计出来的轨迹最终的最终那个点和目标点坐标之间的夹角
#    当靠近目标点时，如果速度不减会超过目标点，导致角度评价得分很低
def eval_heading(trajectory, goal):
    """方位角评价函数
    评估在当前采样速度下产生的轨迹终点位置方向与目标点连线的夹角的误差
    """
    for tra in trajectory:
        if cal_distance(tra[0], tra[1], goal[0], goal[1]) <= 0.1:
            return math.pi
    dx = goal[0] - trajectory[-1][0]
    dy = goal[1] - trajectory[-1][1]
    error_angle = math.atan2(dy, dx)
    cost_angle = error_angle - trajectory[-1][2]
    cost = math.pi - abs(cost_angle)
    return cost


# 越大越好
# 速度评价函数，可以用模拟轨迹末端位置的线速度的大小来表示，有问题，不能只看这个速度的大小而不看方向
#  这里需要优化，速度评价函数最后的输出
def eval_velocity(trajectory):
    return trajectory[-1][3]


# 轨迹评价函数,评价越高，轨迹越优
# 传入当前机器人的状态[x,y,yaw,v,w]，目标点坐标goal[x,y], 其他三个机器人的坐标[[x0,y0],[x1,y1],[x2,y2]], 当前这个机器人是否背着物品
# 返回最优控制量 [v,w]
def dwa(state, goal, obstacle, is_carried):
    G_max = float('-inf')  # 最优评价
    control_opt = [0., 0.]  # 最优控制
    dynamic_window_vel = cal_dwv(is_carried, obstacle, [state[0], state[1]], state[3], state[4])  # 计算速度空间
    sum_heading, sum_dist, sum_vel = 0, 0, 0  # 统计全部采样轨迹的各个评价之和，便于评价的归一化
    # 归一化
    v_sample = (dynamic_window_vel[1] - dynamic_window_vel[0]) / 10
    w_sample = (dynamic_window_vel[3] - dynamic_window_vel[2]) / 10
    for v in np.arange(dynamic_window_vel[0], dynamic_window_vel[1], v_sample):
        for w in np.arange(dynamic_window_vel[2], dynamic_window_vel[3], w_sample):
            trajectory = trajectory_predict(state, v, w)
            heading_eval = alpha * eval_heading(trajectory, goal)
            dist_eval = beta * eval_distance(trajectory, obstacle)
            vel_eval = gamma * eval_velocity(trajectory)
            sum_vel += vel_eval
            sum_dist += dist_eval
            sum_heading += heading_eval
    # test_write_file("该速度区间归一化参数分别sum_heading:{}, sum_dist:{}, sum_vel为是：{}".format(sum_heading, sum_dist, sum_vel))
    # 在速度空间中按照预先设定的分辨率采样，
    for v in np.arange(dynamic_window_vel[0], dynamic_window_vel[1], v_sample):
        for w in np.arange(dynamic_window_vel[2], dynamic_window_vel[3], w_sample):
            # test_write_file("v:{}，w:{}".format(v, w))
            trajectory = trajectory_predict(state, v, w)  # 第2步--轨迹推算
            heading_eval = alpha * eval_heading(trajectory, goal) / sum_heading
            dist_eval = beta * eval_distance(trajectory, obstacle) / sum_dist
            vel_eval = gamma * eval_velocity(trajectory) / sum_vel
            G = heading_eval + dist_eval + vel_eval  # 第3步--轨迹评价
            # test_write_file("这个速度方案：{}的得分是：{}".format([v, w], G))
            # test_write_file(
            #     "该速度区间各项打分分别为：角度：{} 距离：{} 速度：{}".format(heading_eval, dist_eval, vel_eval))
            if G_max <= G:
                G_max = G
                control_opt = [v, w]
    # test_write_file("此速度区间得到的速度参数：{}".format(control_opt))
    return control_opt


# 计算这一帧所有这个机器人应该执行的指令
#    机器人当前的坐标，机器人当前的角度,目标工作台的坐标
#    输出的是这一帧的这个机器人线速度和角度
use_dwa = True  # 通过这个来设置是否使用dwa


def cal_instruct(is_carry, robot_loc, robot_angle, bench_loc, robot_id):
    # test_write_file('已经进入cal_instruct:{}'.format(frame_id))
    if use_dwa:
        # 得到当前id机器人的状态
        #  首先是当前机器人的线速度
        n_line_speed = math.sqrt(n_robots[robot_id][5][0] ** 2 + n_robots[robot_id][5][1] ** 2)
        m_state = [n_robots[robot_id][7][0],
                   n_robots[robot_id][7][1],
                   n_robots[robot_id][6],
                   n_line_speed,
                   n_robots[robot_id][4]]
        goal = bench_loc
        other_robot_loc = []
        for i in range(4):
            if i != robot_id:
                other_robot_loc.append(n_robots[i][7])
        is_carried = is_carry
        line_v, w = dwa(m_state, goal, other_robot_loc, is_carried)
        # test_write_file("当前是机器人{}坐标是：{}，目标是坐标是{}, 当前机器人的方位角：{}，速度矢量：{}".format(robot_id,n_robots[robot_id][7], bench_loc,
        #                                                                      n_robots[robot_id][6],
        #                                                                      n_robots[robot_id][5]))
        # test_write_file("dwa给定的速度，角速度：{}，{}".format(line_v, w))
        return [line_v, w]

    r_x, r_y = robot_loc[0], robot_loc[1]
    b_x, b_y = bench_loc[0], bench_loc[1]

    r2b = [b_x - r_x, b_y - r_y]  # 机器人指向工作台的向量，目标就是把机器人的速度矢量掰过来
    r2b_a = math.atan2(r2b[1], r2b[0])  # 当前机器人与目标工作台向量与x轴正方向的夹角

    distance = math.sqrt((r_x - b_x) ** 2 + (r_y - b_y) ** 2)  # 当前机器人与工作台的距离

    n_line_speed = 6
    n_angle_speed = 0

    if distance <= 1:
        n_line_speed = 2 + (distance) ** 2

    # if abs(robot_angle - r2b_a) > math.pi / 2:
    #     n_angle_speed = 2

    or_angle_value = abs(robot_angle - r2b_a) * 50

    if r2b_a >= 0 and robot_angle >= 0:
        if robot_angle > r2b_a:
            n_angle_speed = -1 * or_angle_value
        elif robot_angle < r2b_a:
            n_angle_speed = or_angle_value
    elif r2b_a < 0 and robot_angle < 0:
        if robot_angle > r2b_a:
            n_angle_speed = -or_angle_value
        elif robot_angle < r2b_a:
            n_angle_speed = or_angle_value
    elif r2b_a < 0 and robot_angle > 0:
        if abs(r2b_a) + abs(robot_angle) < math.pi:
            n_angle_speed = -or_angle_value
        else:
            n_angle_speed = or_angle_value
    else:
        if abs(r2b_a) + abs(robot_angle) < math.pi:
            n_angle_speed = or_angle_value
        else:
            n_angle_speed = -or_angle_value
    # 尝试在背着物品的时候如果速度向量超过图，则将速度设置成负的
    # if is_carry:
    #     v_v = [r_x + n_line_speed * math.cos(robot_angle), r_y + n_line_speed * math.sin(robot_angle)]
    #     v_v = [i*0.3 for i in v_v]
    #     if v_v[0] < 0 or v_v[0] > 50 or v_v[1] < 0 or v_v[1] > 50:
    #         n_angle_speed = -2
    return [n_line_speed, n_angle_speed]


# 帮助函数
# 传进来工作台类型和原材料格子状态，返回这个工作台差的材料类型
each_bench_materials = {4: {1, 2}, 5: {1, 3}, 6: {2, 3}, 7: {4, 5, 6}, 8: {7}, 9: {1, 2, 3, 4, 5, 6, 7}}  # 方便判断缺不缺材料


def lack_which_material(bench_type, material_status):
    had_material = set()
    for i in range(10):
        if (material_status >> i) & 1 == 1:
            had_material.add(i)
    lack_material = each_bench_materials[bench_type] - had_material

    return list(lack_material), len(had_material)


# 帮助函数
# 传进来两个坐标，返回这两个坐标之间的距离
def cal_distance(x1, y1, x2, y2):
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


# 对于每一帧，初始化五个参数，分别记录：
# 1、type_lack,全场每一种物品缺多少,0就是不缺
# 2、robot_carry, 每一个机器人身上都携带着什么类型的物品，没有就是-1
# 3、each_lack, 对于缺的物品种类，按照种类分成若干份，记录是哪个工作台缺的，以及这个工作台的坐标，以及这个工作台现在已经有多少材料
# 4、done_bench, 对于已经有成品的工作台，按照成品的种类将这些工作台分类，记录工作台id和坐标
# 5、each_lack_num, 每一种材料缺的数量-有机器人正在运输的数量，得到每一种材料真正缺少的数量
def init_frame():
    type_lack = [0] * 8  # 一共其中物品，分别用索引表示, 索引0并不存在，永远是0
    each_lack = defaultdict(lambda: [])
    done_bench = defaultdict(lambda: [])
    for bench in n_benches:
        # 工作台123不需要原材料, 但是可能有成品
        if bench[1] in [1, 2, 3]:
            # 如果有产品
            if bench[5] == 1:
                # 对于工作台123，工作台类型就是产品的类型
                done_bench[bench[1]].append([bench[0], bench[2]])  # 记录工作台id和坐标
            continue
        # test_write_file(bench)
        lack_m, had_len = lack_which_material(bench[1], bench[4])  # 我自己设置的bench的存储方式比官方的在第一位多一个id
        for m in lack_m:
            type_lack[m] += 1
            # 注意bench[2]本来就是个列表[x,y], had_len就是这个工作台已经有材料的数量
            each_lack[m].append([bench[0], bench[2], had_len])
            # 循环能到这里说明不是123，89又没有产品
        if bench[1] not in [8, 9]:
            if bench[5] == 1:
                done_bench[bench[1]].append([bench[0], bench[2]])
    robot_carry = [0] * 10  # 9种材料，哪些被机器人带着的

    each_lack_num = [0] * 8  # 1-7，  7种材料分别实际还缺多少

    # test_write_file('each_lack: {}'.format(each_lack))
    # 初始化就是不管当前正在运输的材料的数量，只看缺的
    for lack_type, lack_num in each_lack.items():
        each_lack_num[lack_type] = len(lack_num)

    # 记录机器人一共携带了多少个什么类型的材料
    for ind, robot in enumerate(n_robots):
        robot_carry[robot[1]] += 1

    # 将缺少的材料的个数-已经正在运输中的这个材料的个数
    for ind, lack_num in enumerate(robot_carry):
        if lack_num != 0 and ind != 0:
            each_lack_num[ind] -= lack_num
    # if frame_id % 10 == 0:
    #     test_write_file(each_lack_num)
    return type_lack, robot_carry, each_lack, done_bench, each_lack_num


def task_process():
    # 返回的是每个机器人应该进行的操作，包括线速度，角速度，买卖
    each_robot_act = [[0, 0, -1] for _ in range(4)]  # 0，0表示线速度和角速度，0是买1是卖，-1是不进行买卖操作
    each_not_carry_robot_toward_bench = [-1] * 4  # 所有身上没有背着东西的机器人准备去的工作台序号，-1表示没有
    each_carry_robot_toward_bench = [-1] * 4  # 所有身上带着东西的机器人准备去的工作台序号，-1表示没有
    # 分别对每一个机器人进行操作，机器人id是0123
    # 首先是0号机器人
    #    如果0号机器人没有携带
    if n_robots[0][1] == 0:
        # 如果全场没有缺少的，直接把速度和角速度降到0,也不用进行买卖
        if sum(n_each_lack_num) == 0:
            each_robot_act[0] = [0, 0, -1]
        # 如果全场有缺的，把所有生产好了的而且是缺的物品所在工作台按照与0号机器人之间的距离排队
        else:
            # 首先把所有缺的物品对应的已经生产好了的工作台放到列表中，注意，可能这个列表位空。因为可能这个缺的物品一个都没有生产出来
            l_d_m = []  # 缺少的但是已经做好了的物品对应的工作台，这个工作台的与1号机器人的距离和这个工作台的id
            # test_write_file('n_done_bench: {}'.format(n_done_bench))
            # test_write_file('n_each_lack_num: {}'.format(n_each_lack_num))
            for k, v in n_done_bench.items():
                if n_each_lack_num[k] != 0:
                    # 因为n_done_bench这个字典的值是列表类型，所以需要遍历一个个取
                    for bench in v:
                        l_d_m.append([cal_distance(n_robots[0][7][0],
                                                   n_robots[0][7][1],
                                                   bench[1][0],
                                                   bench[1][1]), bench[0]])
            # test_write_file('缺少的但是已经做好了的物品: ')
            # test_write_file(l_d_m)
            # 如果已经有缺少的成品被生产出来了在进行下面的操作，否则就直接不改变默认的速度设定，也就是全0
            if l_d_m:
                l_d_m.sort()  # 按照距离从小到大排序，取第一个
                # 如果距离最近的工作台已经可以交易，就保持速度为0进行购买操作
                if l_d_m[0][1] == n_robots[0][0]:
                    each_robot_act[0][2] = 0  # 购买
                    n_robots[0][1] = n_benches[l_d_m[0][1]][1]  # 购买之后机器人1身上携带的就是这个材料
                else:
                    # r_instruct_0是计算出来的0号机器人需要执行的命令
                    r_instruct_0 = cal_instruct(0,
                                                n_robots[0][7],
                                                n_robots[0][6],
                                                n_benches[l_d_m[0][1]][2],
                                                0)
                    each_robot_act[0][0] = r_instruct_0[0]  # 线速度
                    each_robot_act[0][1] = r_instruct_0[1]  # 角速度
                    each_not_carry_robot_toward_bench[0] = n_benches[l_d_m[0][1]][0]  # 设置0号机器人要去的工作台，防止后面抢
                    n_each_lack_num[n_benches[l_d_m[0][1]][1]] -= 1  # 0号机器人要去这个工作台生产的材料需求-1

    # 如果0号机器人身上背着一个东西
    else:
        # 0号机器人身上有，说明一定有工作台需要
        need_0_type_m_benches = []  # 所有需要0号机器人背上材料的工作台[[add_weight_distance, bench_id]],加权距离，此工作台id
        for bench in n_each_lack[n_robots[0][1]]:
            need_0_type_m_benches.append([cal_distance(n_robots[0][7][0],
                                                       n_robots[0][7][1],
                                                       bench[1][0],
                                                       bench[1][1]) / (bench[2] + 1),
                                          bench[0]])
        need_0_type_m_benches.sort()  # 按照加权距离进行排序
        # 判断排在第一位的工作台是否可以与机器人0交易，如果可以直接卖
        if n_robots[0][0] == need_0_type_m_benches[0][1]:
            each_robot_act[0][2] = 1  # 卖出
            n_robots[0][1] = 0  # 卖了之后0号机器人身上就没有了
        # 如果这个机器人不能与目标工作台交易，则向他移动
        else:
            target_bench_id = need_0_type_m_benches[0][1]
            r_instruct_0 = cal_instruct(1, n_robots[0][7], n_robots[0][6], n_benches[target_bench_id][2], 0)
            each_robot_act[0][0] = r_instruct_0[0]  # 线速度
            each_robot_act[0][1] = r_instruct_0[1]  # 角速度
            each_carry_robot_toward_bench[0] = target_bench_id
    # 机器人0的指令已经设置完毕，下面是机器人1的指令

    # 如果机器人1没有携带物品
    if n_robots[1][1] == 0:
        # 如果全场没有缺少的，直接把速度和角速度降到0,也不用进行买卖
        if sum(n_each_lack_num) == 0:
            each_robot_act[1] = [0, 0, -1]
        # 如果全场有缺的，把所有生产好了的而且是缺的物品所在工作台按照与1号机器人之间的距离排队
        else:
            # 首先把所有缺的物品对应的已经生产好了的工作台放到列表中，注意，可能这个列表位空。因为可能这个缺的物品一个都没有生产出来
            l_d_m = []  # 缺少的而且已经做好了的物品对应的工作台，里面存放这个工作台与1号机器人的距离和这个工作台的id
            for k, v in n_done_bench.items():
                if n_each_lack_num[k] != 0:
                    # 因为n_done_bench这个字典的值是列表类型，所以需要遍历一个个取
                    for bench in v:
                        l_d_m.append([cal_distance(n_robots[1][7][0],
                                                   n_robots[1][7][1],
                                                   bench[1][0],
                                                   bench[1][1]), bench[0]])
            # 不为空，即有做好了的成品
            if l_d_m:
                l_d_m.sort()  # 按照距离从小到大排序，取第一个与0号机器人相同情况下要去的工作台不同的工作台
                for dis, bench_id in l_d_m:
                    if bench_id != each_not_carry_robot_toward_bench[0]:
                        each_not_carry_robot_toward_bench[1] = bench_id
                        break
                # 如果距离最近的工作台已经可以交易，就保持速度为0进行购买操作
                if each_not_carry_robot_toward_bench[1] == n_robots[1][0]:
                    each_robot_act[1][2] = 0  # 购买
                    n_robots[1][1] = n_benches[each_not_carry_robot_toward_bench[1]][1]  # 购买之后，1号机器人携带的就是这个了
                # 当前不能交易，则向着目标工作台移动
                else:
                    # r_instruct_1是计算出来的1号机器人需要执行的命令
                    r_instruct_1 = cal_instruct(0, n_robots[1][7],
                                                n_robots[1][6],
                                                n_benches[each_not_carry_robot_toward_bench[1]][2],
                                                1)
                    each_robot_act[1][0] = r_instruct_1[0]  # 线速度
                    each_robot_act[1][1] = r_instruct_1[1]  # 角速度
                    n_each_lack_num[n_benches[each_not_carry_robot_toward_bench[1]][1]] -= 1  # 1号机器人要去这个工作台生产的材料需求-1

    # 如果机器人1身上带着物品
    else:
        # 1号机器人身上有，说明一定有工作台需要
        need_1_type_m_benches = []  # 所有需要1号机器人背上材料的工作台[[add_weight_distance, bench_id]],加权距离，此工作台id
        for bench in n_each_lack[n_robots[1][1]]:
            need_1_type_m_benches.append([cal_distance(n_robots[1][7][0],
                                                       n_robots[1][7][1],
                                                       bench[1][0],
                                                       bench[1][1]) / (bench[2] + 1),
                                          bench[0]])
        need_1_type_m_benches.sort()  # 按照加权距离进行排序
        # 判断身上背的东西与0号机器人身上背的东西是否相同，这一步一定需要，因为如果背的东西不同，哪怕朝向同一个工作台也无所谓的
        #    相同
        if n_robots[1][1] == n_robots[0][1]:
            for i in need_1_type_m_benches:
                if i[1] != each_carry_robot_toward_bench[0]:
                    each_carry_robot_toward_bench[1] = i[1]
                    break
        # 与0号机器人携带的东西不同
        else:
            each_carry_robot_toward_bench[1] = need_1_type_m_benches[0][1]  # 那目标bench就是加权排序后的第一个
        # 能卖就卖
        if n_robots[1][0] == each_carry_robot_toward_bench[1]:
            each_robot_act[1][2] = 1
            n_robots[1][1] = 0  # 卖掉之后就没有携带物品了
        # 不能卖，向它靠近
        else:
            target_bench_id = each_carry_robot_toward_bench[1]
            r_instruct_1 = cal_instruct(1, n_robots[1][7], n_robots[1][6], n_benches[target_bench_id][2], 1)
            each_robot_act[1][0] = r_instruct_1[0]  # 线速度
            each_robot_act[1][1] = r_instruct_1[1]  # 角速度
    # 1号机器人处理完毕，下面是2号机器人

    # 如果2号机器人没有携带物品
    if n_robots[2][1] == 0:
        # 如果全场没有缺少的，直接把速度和角速度降到0,也不用进行买卖
        if sum(n_each_lack_num) == 0:
            each_robot_act[2] = [0, 0, -1]
        # 如果全场有缺的，把所有生产好了的而且是缺的物品所在工作台按照与2号机器人之间的距离排队
        else:
            # 首先把所有缺的物品对应的已经生产好了的工作台放到列表中，注意，可能这个列表位空。因为可能这个缺的物品一个都没有生产出来
            l_d_m = []  # 缺少的而且已经做好了的物品对应的工作台，里面存放这个工作台与2号机器人的距离和这个工作台的id
            for k, v in n_done_bench.items():
                if n_each_lack_num[k] != 0:
                    # 因为n_done_bench这个字典的值是列表类型，所以需要遍历一个个取
                    for bench in v:
                        l_d_m.append([cal_distance(n_robots[2][7][0],
                                                   n_robots[2][7][1],
                                                   bench[1][0],
                                                   bench[1][1]), bench[0]])
            # 不为空，即有做好了的成品, 如果为空的话其实不用操作的，还是默认的指令就好了
            if l_d_m:
                l_d_m.sort()  # 按照距离从小到大排序，取第一个与0号机器人1号机器人相同情况下要去的工作台都不同的工作台
                for dis, bench_id in l_d_m:
                    if bench_id != each_not_carry_robot_toward_bench[0] and bench_id != \
                            each_not_carry_robot_toward_bench[1]:
                        each_not_carry_robot_toward_bench[2] = bench_id
                        break
                # 如果距离最近的工作台已经可以交易，就保持速度为0进行购买操作
                if each_not_carry_robot_toward_bench[2] == n_robots[2][0]:
                    each_robot_act[2][2] = 0  # 购买
                    n_robots[2][1] = n_benches[each_not_carry_robot_toward_bench[2]][1]  # 购买之后就需要改变2号机器人携带的物品的状态了
                # 当前不能交易，则向着目标工作台移动
                else:
                    # r_instruct_2是计算出来的2号机器人需要执行的命令
                    r_instruct_2 = cal_instruct(0, n_robots[2][7],
                                                n_robots[2][6],
                                                n_benches[each_not_carry_robot_toward_bench[2]][2], 2)
                    each_robot_act[2][0] = r_instruct_2[0]  # 线速度
                    each_robot_act[2][1] = r_instruct_2[1]  # 角速度
                    n_each_lack_num[n_benches[each_not_carry_robot_toward_bench[2]][1]] -= 1  # 2号机器人要去这个工作台生产的材料需求-1
    # 如果2号机器人携带了物品
    else:
        # 2号机器人身上有，说明一定有工作台需要
        need_2_type_m_benches = []  # 所有需要2号机器人背上材料的工作台[[add_weight_distance, bench_id]],加权距离，此工作台id
        for bench in n_each_lack[n_robots[2][1]]:
            need_2_type_m_benches.append([cal_distance(n_robots[2][7][0],
                                                       n_robots[2][7][1],
                                                       bench[1][0],
                                                       bench[1][1]) / (bench[2] + 1),  # 注意这里要+1，因为可能为0
                                          bench[0]])
        need_2_type_m_benches.sort()  # 按照加权距离进行排序
        # 判断身上背的东西与0号,1号机器人身上背的东西是否相同，一共有4种情况
        #  与0同与1不同, 找第一个与0不同的工作台
        if n_robots[2][1] == n_robots[0][1] and n_robots[2][1] != n_robots[1][1]:
            for i in need_2_type_m_benches:
                if i[1] != each_carry_robot_toward_bench[0]:
                    each_carry_robot_toward_bench[2] = i[1]
                    break
        # 与0不同与1相同，找到第一个与1不同的工作台
        elif n_robots[2][1] != n_robots[0][1] and n_robots[2][1] == n_robots[1][1]:
            for i in need_2_type_m_benches:
                if i[1] != each_carry_robot_toward_bench[1]:
                    each_carry_robot_toward_bench[2] = i[1]
                    break
        # 与0不同与1也不相同，找到第一个工作台
        elif n_robots[2][1] != n_robots[0][1] and n_robots[2][1] != n_robots[1][1]:
            each_carry_robot_toward_bench[2] = need_2_type_m_benches[0][1]
            # 与0和1都相同,找到第一个与0和1都不同的工作台
        else:
            for i in need_2_type_m_benches:
                if i[1] != each_carry_robot_toward_bench[0] and i[1] != each_carry_robot_toward_bench[1]:
                    each_carry_robot_toward_bench[2] = i[1]
                    break

        # 能卖就卖
        if n_robots[2][0] == each_carry_robot_toward_bench[2]:
            each_robot_act[2][2] = 1
            n_robots[2][1] = 0  # 卖掉之后就没有携带物品了
        # 不能卖，向它靠近
        else:
            target_bench_id = each_carry_robot_toward_bench[2]
            r_instruct_2 = cal_instruct(1, n_robots[2][7], n_robots[2][6], n_benches[target_bench_id][2], 2)
            each_robot_act[2][0] = r_instruct_2[0]  # 线速度
            each_robot_act[2][1] = r_instruct_2[1]  # 角速度
    # 机器人2处理完毕，下面是机器人3

    # 如果机器人3没有携带物品
    if n_robots[3][1] == 0:
        # 如果全场没有缺少的，直接把速度和角速度降到0,也不用进行买卖
        if sum(n_each_lack_num) == 0:
            each_robot_act[3] = [0, 0, -1]
        # 如果全场有缺的，把所有生产好了的而且是缺的物品所在工作台按照与3号机器人之间的距离排队
        else:
            # 首先把所有缺的物品对应的已经生产好了的工作台放到列表中，注意，可能这个列表位空。因为可能这个缺的物品一个都没有生产出来
            l_d_m = []  # 缺少的而且已经做好了的物品对应的工作台，里面存放这个工作台与3号机器人的距离和这个工作台的id
            for k, v in n_done_bench.items():
                if n_each_lack_num[k] != 0:
                    # 因为n_done_bench这个字典的值是列表类型，所以需要遍历一个个取
                    for bench in v:
                        l_d_m.append([cal_distance(n_robots[3][7][0],
                                                   n_robots[3][7][1],
                                                   bench[1][0],
                                                   bench[1][1]), bench[0]])

            # 不为空，即有做好了的成品, 如果为空的话其实不用操作的，还是默认的指令就好了
            if l_d_m:
                l_d_m.sort()  # 按照距离从小到大排序，取第一个与0号机器人1号机器人2号相同情况下要去的工作台都不同的工作台
                for dis, bench_id in l_d_m:
                    if bench_id != each_not_carry_robot_toward_bench[0] and bench_id != \
                            each_not_carry_robot_toward_bench[1] and bench_id != each_not_carry_robot_toward_bench[2]:
                        each_not_carry_robot_toward_bench[3] = bench_id
                        break
                # 如果距离最近的工作台已经可以交易，就保持速度为0进行购买操作
                if each_not_carry_robot_toward_bench[3] == n_robots[3][0]:
                    each_robot_act[3][2] = 0  # 购买
                    n_robots[3][1] = n_benches[each_not_carry_robot_toward_bench[3]][1]  # 购买之后就需要改变2号机器人携带的物品的状态了
                # 当前不能交易，则向着目标工作台移动
                else:
                    # r_instruct_3是计算出来的3号机器人需要执行的命令
                    r_instruct_3 = cal_instruct(0, n_robots[3][7],
                                                n_robots[3][6],
                                                n_benches[each_not_carry_robot_toward_bench[3]][2],
                                                3)
                    each_robot_act[3][0] = r_instruct_3[0]  # 线速度
                    each_robot_act[3][1] = r_instruct_3[1]  # 角速度
                    n_each_lack_num[n_benches[each_not_carry_robot_toward_bench[3]][1]] -= 1  # 3号机器人要去这个工作台生产的材料需求-1
    # 如果3号机器人携带了物品
    else:
        # 3号机器人身上有，说明一定有工作台需要
        need_3_type_m_benches = []  # 所有需要3号机器人背上材料的工作台[[add_weight_distance, bench_id]],加权距离，此工作台id
        for bench in n_each_lack[n_robots[3][1]]:
            need_3_type_m_benches.append([cal_distance(n_robots[3][7][0],
                                                       n_robots[3][7][1],
                                                       bench[1][0],
                                                       bench[1][1]) / (bench[2] + 1),  # 注意这里要+1，因为可能为0
                                          bench[0]])
        need_3_type_m_benches.sort()  # 按照加权距离进行排序
        # 判断身上背的东西与0号,1号，2号机器人身上背的东西是否相同，一共有8种情况
        #  1,与012都不同, 找第一个工作台
        if n_robots[3][1] != n_robots[0][1] and n_robots[3][1] != n_robots[1][1] and n_robots[3][1] != n_robots[2][1]:
            each_carry_robot_toward_bench[3] = need_3_type_m_benches[0][1]
        #  2与0同，12不同，找到第一个与0不同的
        elif n_robots[3][1] == n_robots[0][1] and n_robots[3][1] != n_robots[1][1] and n_robots[3][1] != n_robots[2][1]:
            for i in need_3_type_m_benches:
                if i[1] != each_carry_robot_toward_bench[0]:
                    each_carry_robot_toward_bench[3] = i[1]
                    break
        #  3,与1同，02不同，找到第一个与1不同的
        elif n_robots[3][1] != n_robots[0][1] and n_robots[3][1] == n_robots[1][1] and n_robots[3][1] != \
                n_robots[2][1]:
            for i in need_3_type_m_benches:
                if i[1] != each_carry_robot_toward_bench[1]:
                    each_carry_robot_toward_bench[3] = i[1]
                    break
        #  4,与2同，01不同，找到第一个与2不同的
        elif n_robots[3][1] != n_robots[0][1] and n_robots[3][1] != n_robots[1][1] and n_robots[3][1] == \
                n_robots[2][1]:
            for i in need_3_type_m_benches:
                if i[1] != each_carry_robot_toward_bench[2]:
                    each_carry_robot_toward_bench[3] = i[1]
                    break
        #  5,与01同，2不同，找到第一个与01不同的
        elif n_robots[3][1] == n_robots[0][1] and n_robots[3][1] == n_robots[1][1] and n_robots[3][1] != \
                n_robots[2][1]:
            for i in need_3_type_m_benches:
                if i[1] != each_carry_robot_toward_bench[0] and i[1] != each_carry_robot_toward_bench[1]:
                    each_carry_robot_toward_bench[3] = i[1]
                    break
        #  6,与02同，1不同，找到第一个与02不同的
        elif n_robots[3][1] == n_robots[0][1] and n_robots[3][1] != n_robots[1][1] and n_robots[3][1] == \
                n_robots[2][1]:
            for i in need_3_type_m_benches:
                if i[1] != each_carry_robot_toward_bench[0] and i[1] != each_carry_robot_toward_bench[2]:
                    each_carry_robot_toward_bench[3] = i[1]
                    break
        #  7,与12同，0不同，找到第一个与12不同的
        elif n_robots[3][1] != n_robots[0][1] and n_robots[3][1] == n_robots[1][1] and n_robots[3][1] == \
                n_robots[2][1]:
            for i in need_3_type_m_benches:
                if i[1] != each_carry_robot_toward_bench[1] and i[1] != each_carry_robot_toward_bench[2]:
                    each_carry_robot_toward_bench[3] = i[1]
                    break
        #  8,都相同，找与012都不相同的第一个
        else:
            for i in need_3_type_m_benches:
                if i[1] != each_carry_robot_toward_bench[0] and i[1] != each_carry_robot_toward_bench[1] and i[1] != \
                        each_carry_robot_toward_bench[2]:
                    each_carry_robot_toward_bench[3] = i[1]
                    break
        # 能卖就卖
        if n_robots[3][0] == each_carry_robot_toward_bench[3]:
            each_robot_act[3][2] = 1
            n_robots[3][1] = 0  # 卖掉之后就没有携带物品了
        # 不能卖，向它靠近
        else:
            target_bench_id = each_carry_robot_toward_bench[3]
            r_instruct_3 = cal_instruct(1, n_robots[3][7], n_robots[3][6], n_benches[target_bench_id][2], 3)
            each_robot_act[3][0] = r_instruct_3[0]  # 线速度
            each_robot_act[3][1] = r_instruct_3[1]  # 角速度
            # test_write_file(str(frame_id) + '老四判定' + str(r_instruct_3))
    return each_robot_act


# 初始化地图，得到所有工作台和机器人的坐标以及序号
def read_map_util_ok():
    while input() != "OK":
        pass


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


if __name__ == '__main__':
    # 读取机器人以及工作台的初始位置信息
    read_map_util_ok()
    # 每一次与判题器交互后都要输出ok并flush
    finish()

    while True:
        # 第一个必须由外面的循环读取，否则不能判断是否已经结束
        start_time = time.perf_counter()
        line = sys.stdin.readline()
        if not line:
            break
        parts = line.split(' ')
        frame_id = int(parts[0])
        # test_write_file(frame_id)
        # 读取信息并得到当前工作台和机器人的状态信息
        n_benches, n_robots = read_status()
        # 处理好每一帧需要的4个数据
        n_type_lack, n_robot_carry, n_each_lack, n_done_bench, n_each_lack_num = init_frame()
        # 这一帧每个机器人应该执行的操作
        n_each_robot_act = task_process()
        # test_write_file(n_each_robot_act)
        sys.stdout.write('%d\n' % frame_id)
        # 给出帧数，现在因该显示的时间,10帧输出一个
        # if frame_id % 10 == 0:
        #     test_write_file(cal_time(frame_id))
        for ind, act in enumerate(n_each_robot_act):
            sys.stdout.write('forward %d %f\n' % (ind, act[0]))
            sys.stdout.write('rotate %d %f\n' % (ind, act[1]))
            # 如果最后只剩三秒了，就别买了
            if act[2] == 0 and frame_id <= 8940:
                sys.stdout.write('buy %d \n' % ind)
            elif act[2] == 1:
                sys.stdout.write('sell %d \n' % ind)
        end_time = time.perf_counter()
        # test_write_file('这一帧使用时间为：{}ms'.format((end_time - start_time) * 1000))
        finish()
