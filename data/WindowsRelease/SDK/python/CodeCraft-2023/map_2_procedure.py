#!/bin/bash
import math
import sys


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
    return m_benches, m_robots


# 帮助函数
# 传进来两个坐标，返回这两个坐标之间的距离
def cal_distance(x1, y1, x2, y2):
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


def cell2loc(cell_loc):
    """
    :param x: 方格行数
    :param y: 方格列数
    :return: 方格的中心坐标

    """
    cell_x, cell_y = cell_loc
    x = 0.25 + 0.5 * cell_x
    y = 0.25 + 0.5 * cell_y
    return [x, y]


# 给定两个机器人的id，判断在接下来的一秒内他们是否会相撞
def simple_dwa(r_id1, r_id2):
    # 简单的运动学模型，xy坐标，速度，角速度，与x轴正方向的夹角
    # 返回20ms后的xy坐标，速度，角速度，与x轴正方向的夹角
    def KinematicModel(x, y, v, w, theta):
        x = x + v * math.cos(theta) * 0.02
        y = y + v * math.sin(theta) * 0.02
        theta = theta + w * 0.02
        return x, y, v, w, theta

    # 计算机器人1在1s，也就是50个20ms之内的额所有状态点
    x1, y1 = n_robots[r_id1][7]
    v1 = math.sqrt(n_robots[r_id1][5][0] ** 2 + n_robots[r_id1][5][1] ** 2)
    w1 = n_robots[r_id1][4]
    theta1 = n_robots[r_id1][6]
    r1_locations = [[x1, y1]]
    for i in range(150):
        x1, y1, v1, w1, theta1 = KinematicModel(x1, y1, v1, w1, theta1)
        r1_locations.append([x1, y1])

    # 计算机器人2在1s，也就是50个20ms之内的额所有状态点
    x2, y2 = n_robots[r_id2][7]
    v2 = math.sqrt(n_robots[r_id2][5][0] ** 2 + n_robots[r_id2][5][1] ** 2)
    w2 = n_robots[r_id2][4]
    theta2 = n_robots[r_id2][6]
    r2_locations = [[x2, y2]]
    for i in range(150):
        x2, y2, v2, w2, theta2 = KinematicModel(x2, y2, v2, w2, theta2)
        r2_locations.append([x2, y2])

    # 计算是否会碰撞，即计算之间的距离最近是否超过了二者半径和
    # 有一个带了东西
    if (n_robots[r_id1][1] != 0 and n_robots[r_id2][1] == 0) or (n_robots[r_id2][1] != 0 and n_robots[r_id1][1] == 0):
        collision_r = 0.98
    # 都没带
    elif n_robots[r_id1][1] == 0 and n_robots[r_id2][1] == 0:
        collision_r = 0.9
    # 都带了
    else:
        collision_r = 1.06
    collision = False
    for i in range(len(r1_locations)):
        if cal_distance(r1_locations[i][0], r1_locations[i][1], r2_locations[i][0], r2_locations[i][1]) <= collision_r:
            collision = True
            break
    return collision


# 如果检测出来要相撞，如何转向
def decide_angle_speed(m_id, nearest_id):
    # 距离最近的那个机器人指向这个机器人的向量
    v_1 = [n_robots[m_id][7][0] - n_robots[nearest_id][7][0],
           n_robots[m_id][7][1] - n_robots[nearest_id][7][1]]
    quadrant = 1
    if v_1[0] >= 0:
        if v_1[1] > 0:
            quadrant = 1
        else:
            quadrant = 4
    else:
        if v_1[1] > 0:
            quadrant = 2
        else:
            quadrant = 3
    # 计算最近机器人指向我的向量与x轴正方向的夹角alpha
    alpha = math.atan2(v_1[1], v_1[0])
    # 离我最近的机器人朝向theta
    theta = n_robots[nearest_id][6]
    re_angle_speed = 0
    if quadrant == 1:
        if -1 * math.pi / 2 <= theta <= alpha:
            re_angle_speed = -3.4
        if alpha <= theta <= math.pi:
            re_angle_speed = 3.4
    elif quadrant == 4:
        if alpha <= theta <= math.pi / 2:
            re_angle_speed = 3.4
        if -1 * math.pi <= theta <= alpha:
            re_angle_speed = -3.4
    elif quadrant == 3:
        if alpha <= theta <= 0:
            re_angle_speed = 3.4
        if -1 * math.pi <= theta <= 0 or math.pi / 2 <= theta <= math.pi:
            re_angle_speed = -3.4
    else:
        if 0 <= theta <= alpha:
            re_angle_speed = -3.4
        elif -1 * math.pi <= theta <= -1 * math.pi / 2 or alpha <= theta <= math.pi:
            re_angle_speed = 3.4
    return re_angle_speed


# 使用原来的的最简单的控制小车的方式
def traditional_instruct(robot_id):
    if each_robot_step[robot_id] < len(each_robot_path[robot_id]):
        x_target, y_target = each_robot_path[robot_id][each_robot_step[robot_id]]
        # 现在机器人的实际坐标
        n_x, n_y = n_robots[robot_id][7][0], n_robots[robot_id][7][1]
        distance_error = math.sqrt(
            (x_target - n_x) ** 2 + (y_target - n_y) ** 2)
        if distance_error <= 0.4:
            each_robot_step[robot_id] += 1
            return 0, 0
        # 当前机器人与目标工作台向量与x轴正方向的夹角
        r2b_a = math.atan2(y_target - n_y, x_target - n_x)
        robot_angle = n_robots[robot_id][6]
        or_angle_value = abs(robot_angle - r2b_a) * 50
        # 默认线速度和角速度
        line_speed = 6.4 if distance_error > 1 else distance_error
        angle_speed = 0
        # 简单角速度约束，防止蛇形走位
        if r2b_a >= 0 and robot_angle >= 0:
            if robot_angle >= r2b_a:
                angle_speed = -1 * or_angle_value
            elif robot_angle < r2b_a:
                angle_speed = or_angle_value
        elif r2b_a <= 0 and robot_angle <= 0:
            if robot_angle >= r2b_a:
                angle_speed = -or_angle_value
            elif robot_angle < r2b_a:
                angle_speed = or_angle_value
        elif r2b_a <= 0 and robot_angle >= 0:
            if abs(r2b_a) + abs(robot_angle) <= math.pi:
                angle_speed = -or_angle_value
            else:
                angle_speed = or_angle_value
        else:
            if abs(r2b_a) + abs(robot_angle) <= math.pi:
                angle_speed = or_angle_value
            else:
                angle_speed = -or_angle_value
        # 距离目标距离对线速度的约束，防止打转
        if abs(robot_angle - r2b_a) >= 0.5:
            line_speed = -0.2

        # 简单的避撞
        # 计算距离我最近的小球的id以及之间的距离
        nearest_robot_id = -1
        nearest_distance = float('inf')
        for r_i in range(4):
            if r_i != robot_id:
                t = cal_distance(n_robots[robot_id][7][0], n_robots[robot_id][7][1], n_robots[r_i][7][0],
                                 n_robots[r_i][7][1])
                if t < nearest_distance:
                    nearest_robot_id = r_i
                    nearest_distance = t
        # 使用简单的避障方式避障
        use_simple_collision_avoidance = True
        if use_simple_collision_avoidance:
            warning_distance = 1.06
            # 防止机器人之间的碰撞
            if nearest_distance <= warning_distance:
                if simple_dwa(robot_id, nearest_robot_id):
                    angle_speed = decide_angle_speed(robot_id, nearest_robot_id)
        return line_speed, angle_speed
    else:
        return 0, 0


# 将四个地图的运动控制完全分开
def cal_instruct_1(robot_id):
    return traditional_instruct(robot_id)


# 每一个机器人被规划出来的路径
# path写的时候是cell，但是会通过cell2loc进行转换，这样会更方便
robot_1_path = [[31, 63], [49, 62], [49, 53],  # 到达5号工作台
                [49, 63], [22, 63], [22, 43], [31, 46], [26, 58],  # 将1送到2号工作台
                [25, 63], [79, 64], [86, 64], [86, 58], [81, 58],  # 到达4号工作台
                [86, 58], [86, 14], [49, 14], [49, 33], [23, 36], [23, 40], [31, 45], [26, 58],  # 送回到2号工作台
                [25, 62], [49, 62], [49, 53],  # 到达5号工作台
                [49, 62], [22, 61], [22, 43], [31, 46], [26, 58],  # 将1送到2号工作台

                [31, 45], [31, 43], [23, 38], [23, 33], [34, 31], [34, 19],  # 将物品5送到6号工作台
                [34, 33], [23, 33], [23, 38], [31, 43], [31, 45], [26, 58],  # 回到工作台2

                [25, 63], [79, 64], [86, 64], [86, 58], [81, 58],  # 到达4号工作台
                [86, 58], [86, 14], [49, 14], [49, 33], [23, 36], [23, 40], [31, 45], [26, 58],  # 送回到2号工作台


                [31, 45], [31, 43], [23, 38], [23, 33], [34, 31], [34, 19],  # 将物品5送到6号工作台，这时候5肯定没有生产好，要让他等
                [34, 33], [23, 33], [23, 38], [31, 43], [31, 45], [26, 58],  # 回到工作台2


                [32, 62], [31, 63], [49, 62], [49, 53],  # 第二遍
                [49, 63], [22, 63], [22, 43], [31, 46], [26, 58],  # 将1送到2号工作台
                [25, 63], [79, 64], [86, 64], [86, 58], [81, 58],  # 到达4号工作台
                [86, 58], [86, 14], [49, 14], [49, 33], [23, 36], [23, 40], [31, 45], [26, 58],  # 送回到2号工作台
                [25, 62], [49, 62], [49, 53],  # 到达5号工作台
                [49, 62], [22, 61], [22, 43], [31, 46], [26, 58],  # 将1送到2号工作台

                [31, 45], [31, 43], [23, 38], [23, 33], [34, 31], [34, 19],  # 将物品5送到6号工作台
                [34, 33], [23, 33], [23, 38], [31, 43], [31, 45], [26, 58],  # 回到工作台2

                [25, 63], [79, 64], [86, 64], [86, 58], [81, 58],  # 到达4号工作台
                [86, 58], [86, 14], [49, 14], [49, 33], [23, 36], [23, 40], [31, 45], [26, 58],  # 送回到2号工作台


                [31, 45], [31, 43], [23, 38], [23, 33], [34, 31], [34, 19],  # 将物品5送到6号工作台，这时候5肯定没有生产好，要让他等
                [34, 33], [23, 33], [23, 38], [31, 43], [31, 45], [26, 58],  # 回到工作台2

                [10, 10]  # 任务完成，避开别碍事
                ]
robot_1_path = [cell2loc(i) for i in robot_1_path]

robot_2_path = [[67, 60], [67, 65], [20, 64], [12, 64], [12, 58], [18, 58],  # 到达1号工作台
                [12, 58], [12, 14], [49, 14], [49, 32], [76, 32], [76, 38], [67, 43], [73, 58],  # 将物品1送回到3号工作台
                [67, 60], [67, 65], [49, 60], [49, 53],  # 到达5号工作台
                [49, 62], [67, 65], [77, 60], [77, 42], [67, 46], [73, 58],  # 将物品1送回到3号工作台

                [67, 60], [67, 65], [20, 64], [12, 64], [12, 58], [18, 58],  # 到达1号工作台
                [12, 58], [12, 14], [49, 14], [49, 32], [76, 32], [76, 38], [67, 43], [73, 58],  # 将物品1送回到3号工作台

                [67, 45], [67, 43], [76, 38], [76, 35], [34, 33], [34, 31], [34, 19],  # 到达6号工作台
                [34, 33], [76, 35], [76, 38], [67, 43], [67, 45], [73, 58],  # 回到3号工作台

                [67, 60], [67, 65], [49, 60], [49, 53],  # 到达5号工作台
                [49, 62], [67, 65], [77, 60], [77, 42], [67, 46], [73, 58],  # 将物品1送回到3号工作台

                [67, 45], [67, 43], [76, 38], [76, 35], [34, 33], [34, 31], [34, 19],  # 到达6号工作台，刚开始肯定没有做好，也是要等
                [34, 33], [76, 35], [76, 38], [67, 43], [67, 45], [73, 58],  # 回到3号工作台


                [67, 60], [67, 65], [20, 64], [12, 64], [12, 58], [18, 58],  # 第二遍
                [12, 58], [12, 14], [49, 14], [49, 32], [76, 32], [76, 38], [67, 43], [73, 58],  # 将物品1送回到3号工作台
                [67, 60], [67, 65], [49, 60], [49, 53],  # 到达5号工作台
                [49, 62], [67, 65], [77, 60], [77, 42], [67, 46], [73, 58],  # 将物品1送回到3号工作台

                [67, 60], [67, 65], [20, 64], [12, 64], [12, 58], [18, 58],  # 到达1号工作台
                [12, 58], [12, 14], [49, 14], [49, 32], [76, 32], [76, 38], [67, 43], [73, 58],  # 将物品1送回到3号工作台

                [67, 45], [67, 43], [76, 38], [76, 35], [34, 33], [34, 31], [34, 19],  # 到达6号工作台
                [34, 33], [76, 35], [76, 38], [67, 43], [67, 45], [73, 58],  # 回到3号工作台

                [67, 60], [67, 65], [49, 60], [49, 53],  # 到达5号工作台
                [49, 62], [67, 65], [77, 60], [77, 42], [67, 46], [73, 58],  # 将物品1送回到3号工作台

                [67, 45], [67, 43], [76, 38], [76, 35], [34, 33], [34, 31], [34, 19],  # 到达6号工作台，刚开始肯定没有做好，也是要等
                [34, 33], [76, 35], [76, 38], [67, 43], [67, 45], [73, 58],  # 回到3号工作台

                [10, 10]  # 任务完成，避开别碍事
                ]

robot_2_path = [cell2loc(i) for i in robot_2_path]

robot_3_path = [
    [49, 33], [49, 15], [86, 15], [86, 45], [81, 58],  # 到达4号工作台
    [86, 45], [96, 42], [96, 83], [50, 83], [50, 74],  # 回到工作台0

    [50, 83], [12, 83], [12, 58], [18, 58],  # 到达1号工作台
    [12, 45], [12, 40], [1, 40], [1, 83], [50, 83], [50, 74],  # 回到0号工作台

    [50, 83], [96, 83], [96, 42], [86, 45], [81, 58],  # 到达4号工作台
    [86, 45], [96, 42], [96, 83], [50, 83], [50, 74],  # 回到工作台0
    [50, 83], [39, 83], [39, 63], [22, 63], [22, 33], [34, 33], [34, 31], [34, 19],  # 满载0号到6号工作台
    [34, 19], [34, 31], [34, 33], [22, 33], [22, 63], [49, 63], [50, 66], [50, 74],  # 空6到0

    [49, 66], [49, 63], [20, 63], [12, 60], [12, 58], [18, 58],  # 到达1号工作台
    [12, 45], [12, 40], [1, 40], [1, 83], [50, 83], [50, 74],  # 1号到0号工作台

    [50, 83], [39, 83], [39, 63], [22, 63], [22, 33], [34, 33], [34, 31], [34, 19],  # 满载0号到6号工作台，也是没做好，要等
    [34, 19], [34, 31], [34, 33], [22, 33], [22, 63], [49, 63], [50, 66], [50, 74],  # 空6到0


    # 第二遍
    [50, 83], [96, 83], [96, 42], [86, 45], [81, 58],  # 到达4号工作台
    [86, 45], [96, 42], [96, 83], [50, 83], [50, 74],  # 回到工作台0

    [50, 83], [12, 83], [12, 58], [18, 58],  # 到达1号工作台
    [12, 45], [12, 40], [1, 40], [1, 83], [50, 83], [50, 74],  # 回到0号工作台

    [50, 83], [96, 83], [96, 42], [86, 45], [81, 58],  # 到达4号工作台
    [86, 45], [96, 42], [96, 83], [50, 83], [50, 74],  # 回到工作台0
    [50, 83], [39, 83], [39, 63], [22, 63], [22, 33], [34, 33], [34, 31], [34, 19],  # 满载0号到6号工作台
    [34, 19], [34, 31], [34, 33], [22, 33], [22, 63], [49, 63], [50, 66], [50, 74],  # 空6到0

    [49, 66], [49, 63], [20, 63], [12, 60], [12, 58], [18, 58],  # 到达1号工作台
    [12, 45], [12, 40], [1, 40], [1, 83], [50, 83], [50, 74],  # 1号到0号工作台

    [50, 83], [39, 83], [39, 63], [22, 63], [22, 33], [34, 33], [34, 31], [34, 19],  # 满载0号到6号工作台，也是没做好，要等
    [34, 19], [34, 31], [34, 33], [22, 33], [22, 63], [49, 63], [50, 66], [50, 74],  # 空6到0

    [100, 10]  # 任务完成，避开别碍事
]
robot_3_path = [cell2loc(i) for i in robot_3_path]

robot_4_path = [[34, 33], [34, 31], [34, 25],
                [10, 25],  # 让这个卡住，只有当6号工作台有成品了才移动,3
                [34, 19], [34, 34], [64, 34], [65, 19],  # 满载卖掉7
                [65, 19], [64, 34], [34, 34], [34, 25],  # 进入6号工作台所在的房间待命
                [10, 25],  # 故意卡住 12
                [34, 19], [34, 34], [64, 34], [65, 19],  # 满载卖掉7
                [65, 19], [64, 34], [34, 34], [34, 25],  # 进入6号工作台所在的房间待命
                [10, 25],  # 故意卡住 21
                [34, 19], [34, 34], [64, 34], [65, 19],  # 满载卖掉7
                [65, 19], [64, 34], [34, 34], [34, 25],  # 进入6号工作台所在的房间待命
                [10, 25],  # 故意卡住 24
                ]
robot_4_path = [cell2loc(i) for i in robot_4_path]
each_robot_path = [robot_1_path, robot_2_path, robot_3_path, robot_4_path]
# step就是规划出来的路径的索引，当路径生成后-1将变成0
each_robot_step = [0, 0, 0, 0]
# 每一个机器人目标工作台
robot_1_target_bench = [5, 2, 4, 2, 5, 2,  # 送满2号工作台然后外加一个物品1
                        2, 6,  # 物品5搬到工作台6
                        4, 2,  # 差的一个物品3送给2号工作台
                        2, 6,  # 连着两次将物品5送到6号工作台
                        # 第二遍
                        5, 2, 4, 2, 5, 2,  # 送满2号工作台然后外加一个物品1
                        2, 6,  # 物品5搬到工作台6
                        4, 2,  # 差的一个物品3送给2号工作台
                        2, 6,  # 连着两次将物品5送到6号工作台
                        ]
robot_2_target_bench = [1, 3, 5, 3, 1, 3,
                        3, 6,
                        5, 3,
                        3, 6,
                        # 第二遍
                        1, 3, 5, 3, 1, 3,
                        3, 6,
                        5, 3,
                        3, 6,
                        ]
robot_3_target_bench = [4, 0, 1, 0, 4, 0,
                        0, 6,
                        1, 0,
                        0, 6,

                        4, 0, 1, 0, 4, 0,
                        0, 6,
                        1, 0,
                        0, 6,
                        ]

robot_4_target_bench = [6, 7, 6, 7, 6, 7,
                        6, 7, 6, 7, 6, 7,
                        ]
each_robot_target_bench = [robot_1_target_bench, robot_2_target_bench, robot_3_target_bench, robot_4_target_bench]
each_robot_target_bench_step = [0, 0, 0, 0]


# 空车时直接拿  机器人到成品距离+将成品运送到目标工作台距离最近的那个
#   所有物品的权重一样
def task_process_1():
    # 返回的是每个机器人应该进行的操作，包括线速度，角速度，买卖
    each_robot_act = [[0, 0, -1] for _ in range(4)]  # 0，0表示线速度和角速度，0是买1是卖，-1是不进行买卖操作
    # 这个是3号机器的特判
    if n_benches[6][5] == 1 and each_robot_step[3] in [3, 12, 21, 24]:
        each_robot_step[3] += 1

    for robot_id in range(4):
        if each_robot_target_bench_step[robot_id] >= len(each_robot_target_bench[robot_id]):
            continue
        # 如果机器人身上没有背东西
        if n_robots[robot_id][1] == 0:
            # 如果当前可以与我交易的平台就是我的目标平台，直接购买
            if each_robot_target_bench[robot_id][each_robot_target_bench_step[robot_id]] == n_robots[robot_id][0]:
                # 需要购买时可能当前的工作台并没有制造好，这时候直接continue，原地等待
                if n_benches[n_robots[robot_id][0]][5] == 1:
                    each_robot_act[robot_id][2] = 0  # 购买
                    # 购买之后该机器人的目标工作台后移
                    each_robot_target_bench_step[robot_id] += 1
                else:
                    continue
            # 如果当前工作台并不是我的目标工作台，或者并么有可交易的工作台，则按照轨迹继续运动
            else:
                r_instruct = cal_instruct_1(robot_id)
                each_robot_act[robot_id][0] = r_instruct[0]  # 线速度
                each_robot_act[robot_id][1] = r_instruct[1]  # 角速度
        # 如果当前的机器人身上背着东西
        else:
            # 如果当前可交易的工作台就是我的目标工作台，直接卖出
            if n_robots[robot_id][0] == each_robot_target_bench[robot_id][each_robot_target_bench_step[robot_id]]:
                each_robot_act[robot_id][2] = 1  # 卖出
                # 卖出之后目标工作台后移
                each_robot_target_bench_step[robot_id] += 1
            # 如果还没有到达目标工作台，继续移动
            else:
                r_instruct = cal_instruct_1(robot_id)
                each_robot_act[robot_id][0] = r_instruct[0]  # 线速度
                each_robot_act[robot_id][1] = r_instruct[1]  # 角速度
    return each_robot_act


# 初始化地图，得到所有工作台和机器人的坐标以及序号
def read_map_util_ok():
    s = ''
    t_input = input()
    while t_input != "OK":
        for i in t_input:
            if i != '.':
                s += i
        t_input = input()
    return s


def test_write_file(a):
    with open("test_file.txt", "a", encoding='utf-8') as variable_name:
        variable_name.write(str(a) + '\n')


def finish():
    sys.stdout.write('OK\n')
    sys.stdout.flush()


def map_2_main():
    while True:
        # 第一个必须由外面的循环读取，否则不能判断是否已经结束
        line = sys.stdin.readline()
        if not line:
            break
        parts = line.split(' ')
        global frame_id
        frame_id = int(parts[0])
        # 读取信息并得到当前工作台和机器人的状态信息
        global n_benches, n_robots
        n_benches, n_robots = read_status()
        # 根据每一副地图在不同分配方案上的表现具体确定使用哪种分配方案
        n_each_robot_act = task_process_1()
        # end_time = time.perf_counter()
        sys.stdout.write('%d\n' % frame_id)
        for ind, act in enumerate(n_each_robot_act):
            sys.stdout.write('forward %d %f\n' % (ind, act[0]))
            sys.stdout.write('rotate %d %f\n' % (ind, act[1]))
            # 如果使用的是原始的方案，200ms之前就不买了
            if act[2] == 0:
                sys.stdout.write('buy %d \n' % ind)
            elif act[2] == 1:
                sys.stdout.write('sell %d \n' % ind)
        finish()


if __name__ == '__main__':
    map_2_main()
