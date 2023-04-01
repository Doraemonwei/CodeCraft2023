#!/bin/bash
import math
import sys
from collections import defaultdict

al_dont_need_bench_id = [10, 13]

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

            if bench_id not in al_dont_need_bench_id:
                m_benches.append([bench_id, int(t[0]), [float(t[1]), float(t[2])], int(t[3]), int(t[4]), int(t[5])])
            else:
                m_benches.append([bench_id, int(t[0]), [float(t[1]), float(t[2])], int(t[3]), 126, int(t[5])])
            for j in range(K - 1):
                bench_id += 1
                t = input().split(' ')
                if bench_id not in al_dont_need_bench_id:
                    m_benches.append([bench_id, int(t[0]), [float(t[1]), float(t[2])], int(t[3]), int(t[4]), int(t[5])])
                else:
                    m_benches.append([bench_id, int(t[0]), [float(t[1]), float(t[2])], int(t[3]), 126, int(t[5])])
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
        lack_m, had_len = lack_which_material(bench[1], bench[4])  # 我自己设置的bench的存储方式比官方的在第一位多一个id
        for m in lack_m:
            type_lack[m] += 1
            # bench[0]是这个工作台的id bench[2]是这个工作台的坐标[x,y] had_len就是这个工作台已经有材料的数量
            each_lack[m].append([bench[0], bench[2], had_len])
        # 循环能到这里说明不是123，89又没有产品
        if bench[1] not in [8, 9]:
            if bench[5] == 1:
                done_bench[bench[1]].append([bench[0], bench[2]])
        # test_write_file('done_bench:{}'.format(done_bench))
    robot_carry = [0] * 10  # 9种材料，哪些被机器人带着的

    each_lack_num = [0] * 8  # 1-7，  7种材料分别实际还缺多少

    # 初始化就是不管当前正在运输的材料的数量，只看缺的
    for lack_type, lack_num in each_lack.items():
        each_lack_num[lack_type] = len(lack_num)

    # 记录机器人一共携带了多少个什么类型的材料
    for ind, robot in enumerate(n_robots):
        robot_carry[robot[1]] += 1

    # 将缺少的材料的个数-已经正在运输中的这个材料的个数-已经有机器人准备去拿的材料
    for ind, lack_num in enumerate(robot_carry):
        if lack_num != 0 and ind != 0:
            each_lack_num[ind] -= lack_num
    for ind, will_carry in enumerate(each_not_carry_robot_toward_bench):
        if will_carry != -1 and n_benches[will_carry][1] != 7:
            each_lack_num[n_benches[will_carry][1]] -= 1
    # 如果场上有9工作台，需要对each_lack_num特殊处理
    for i in n_benches:
        if i[1] == 9:
            each_lack_num = [0, 10000, 100000, 10000, 100000, 100000, 100000, 100000, 10000]
            break
    return type_lack, robot_carry, each_lack, done_bench, each_lack_num

# 传入机器人id和假设的放在它身上的货品类型，返回这种假设下它应该去的工作台id
def pre_carried_robot_tar_bench(robot_id, assumption_carry):
    # 如果当前机器人没有携带物品就将假设的这个物品给他
    if n_robots[robot_id][1] == 0:
        material_type = assumption_carry
    else:
        material_type = n_robots[robot_id][1]
    need_robot_id_type_m_benches = []  # 所有需要0号机器人背上材料的工作台[[add_weight_distance, bench_id]],加权距离，工作台id
    # 如果背的是1或2而且类型为4的工作台(id=17)1缺这个玩意，则优先送给17号工作台
    for bench in n_each_lack[material_type]:
        # weight是权重
        weight = 2 ** bench[2] if n_benches[bench[0]][1] != 4 else 100
        need_robot_id_type_m_benches.append([cal_distance(n_robots[robot_id][7][0],
                                                          n_robots[robot_id][7][1],
                                                          bench[1][0],
                                                          bench[1][1]) / weight,
                                             bench[0]])
    need_robot_id_type_m_benches.sort()  # 按照加权距离进行排序
    # 我的目标点应该是哪个工作台
    asumption_target_bench = 10000
    for bench in need_robot_id_type_m_benches:
        flag = False
        for i in range(4):
            if i == robot_id:
                continue
            else:
                # n_robots[i][1] == material_type   不是 n_robots[i][1] == n_robots[robot_id][1] ,因为这里是假设的
                if each_carry_robot_toward_bench[i] == bench[1] and n_robots[i][1] == material_type and \
                        n_benches[bench[1]][1] not in [8, 9]:
                    flag = True
                    break
        if flag:
            continue
        else:
            asumption_target_bench = bench[1]
            break
    if asumption_target_bench == 10000:
        test_write_file(
            '{}\n 需要{}类材料的工作台有：{}，{}选择的工作台是：{}'.format(frame_id, material_type, need_robot_id_type_m_benches, robot_id,
                                                       asumption_target_bench))
    return asumption_target_bench


# 给定两个机器人的id，判断在接下来的两秒秒内他们是否会相撞
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
    for i in range(220):
        x1, y1, v1, w1, theta1 = KinematicModel(x1, y1, v1, w1, theta1)
        r1_locations.append([x1, y1])

    # 计算机器人2在1s，也就是50个20ms之内的额所有状态点
    x2, y2 = n_robots[r_id2][7]
    v2 = math.sqrt(n_robots[r_id2][5][0] ** 2 + n_robots[r_id2][5][1] ** 2)
    w2 = n_robots[r_id2][4]
    theta2 = n_robots[r_id2][6]
    r2_locations = [[x2, y2]]
    for i in range(220):
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


def map_4_instruct(is_carry, robot_loc, robot_angle, bench_loc, robot_id):
    r_x, r_y = robot_loc[0], robot_loc[1]
    b_x, b_y = bench_loc[0], bench_loc[1]
    r2b = [b_x - r_x, b_y - r_y]  # 机器人指向工作台的向量，目标就是把机器人的速度矢量掰过来
    r2b_a = math.atan2(r2b[1], r2b[0])  # 当前机器人与目标工作台向量与x轴正方向的夹角
    distance = math.sqrt((r_x - b_x) ** 2 + (r_y - b_y) ** 2)  # 当前机器人与工作台的距离
    or_angle_value = abs(robot_angle - r2b_a) * 50
    # 默认线速度和角速度
    line_speed = 6
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

    # 地图边界约束，防止撞墙
    if (distance <= 5 or (r_x <= 1 or r_x >= 48 or r_y <= 1 or r_y >= 48)) and abs(
            robot_angle - r2b_a) >= 1.5:
        line_speed = 0.7

    # 距离目标距离对线速度的约束，防止打转
    if distance <= 3.5:
        if abs(robot_angle - r2b_a) >= 1.56:
            line_speed = 1
        if distance <= 0.5:
            if abs(robot_angle - r2b_a) >= 1.56:
                line_speed = 0

    # 计算距离我最近的小球的id以及之间的距离
    nearest_robot_id = -1
    nearest_distance = float('inf')
    for r_i in range(4):
        if r_i != robot_id:
            t = cal_distance(robot_loc[0], robot_loc[1], n_robots[r_i][7][0], n_robots[r_i][7][1])
            if t < nearest_distance:
                nearest_robot_id = r_i
                nearest_distance = t
    # 使用简单的避障方式避障
    use_simple_collision_avoidance = True
    if use_simple_collision_avoidance:
        warning_distance = 5.6
        # 防止机器人之间的碰撞
        if nearest_distance <= warning_distance:
            if simple_dwa(robot_id, nearest_robot_id) and robot_id > nearest_robot_id:
                # angle_speed = decide_angle_speed(robot_id, nearest_robot_id)
                # 这个机器人指向最近那个机器人的向量
                vector_1 = [n_robots[nearest_robot_id][7][0] - robot_loc[0],
                            n_robots[nearest_robot_id][7][1] - robot_loc[1]]
                # 距离最近的那个机器人指向这个机器人的向量
                vector_2 = [robot_loc[0] - n_robots[nearest_robot_id][7][0],
                            robot_loc[1] - n_robots[nearest_robot_id][7][1]]
                # 这个机器人与 这个机器人指向最近那个机器人的向量 之间的夹角余弦
                m_v = n_robots[robot_id][5]  # 这个机器人的速度向量
                t_1 = math.sqrt(m_v[0] ** 2 + m_v[1] ** 2) * math.sqrt(vector_1[0] ** 2 + vector_1[1] ** 2)
                cos_theta_1 = (m_v[0] * vector_1[0] + m_v[1] * vector_1[1]) / t_1 if t_1 != 0 else 0
                # 距离最近的那个机器人与 距离最近的那个机器人指向这个机器人的向量 之间的夹角
                tar_robot_v = n_robots[nearest_robot_id][5]
                t_2 = (math.sqrt(tar_robot_v[0] ** 2 + tar_robot_v[1] ** 2) * math.sqrt(
                    vector_2[0] ** 2 + vector_2[1] ** 2))
                cos_theta_2 = (tar_robot_v[0] * vector_2[0] + tar_robot_v[1] * vector_2[1]) / t_2 if t_2 != 0 else 0
                # 只要有一个小于cos_theta<=0就说明不会撞，所以只考虑都大于0的情况
                if cos_theta_1 >= 0 and cos_theta_2 >= 0:

                    if simple_dwa(robot_id, nearest_robot_id):
                        # if math.acos(cos_theta_1) + math.acos(cos_theta_2) <= (math.pi / 180) * judge_angle:
                        # 小车上下运动和左右运动是不一样的
                        #  前后运动
                        if abs(n_robots[robot_id][5][0]) >= abs(n_robots[robot_id][5][1]):
                            # y0>y1 and x0<x1      y0<y1 and x0>x1  逆时针
                            if (robot_loc[1] >= n_robots[nearest_robot_id][7][1] and robot_loc[0] <=
                                n_robots[nearest_robot_id][7][0]) or ((
                                    robot_loc[1] <= n_robots[nearest_robot_id][7][1] and robot_loc[0] >=
                                    n_robots[nearest_robot_id][7][0])):
                                angle_speed = 3.4
                            else:
                                angle_speed = -3.4
                        # 上下运动
                        else:
                            if (robot_loc[1] > n_robots[nearest_robot_id][7][1] and robot_loc[0] >
                                n_robots[nearest_robot_id][7][0]) or ((
                                    robot_loc[1] < n_robots[nearest_robot_id][7][1] and robot_loc[0] <
                                    n_robots[nearest_robot_id][7][0])):
                                angle_speed = 3.4
                            else:
                                angle_speed = -3.4
    return [line_speed, angle_speed]


# 将四个地图的运动控制完全分开
def cal_instruct_1(is_carry, robot_loc, robot_angle, bench_loc, robot_id):
    return map_4_instruct(is_carry, robot_loc, robot_angle, bench_loc, robot_id)


each_not_carry_robot_toward_bench = [-1] * 4  # 所有身上没有背着东西的机器人准备去的工作台序号，-1表示没有
each_carry_robot_toward_bench = [-1] * 4  # 所有身上带着东西的机器人准备去的工作台序号，-1表示没有


# 空车时直接拿最近的那个
def task_process_1():
    # 返回的是每个机器人应该进行的操作，包括线速度，角速度，买卖
    each_robot_act = [[0, 0, -1] for _ in range(4)]  # 0，0表示线速度和角速度，0是买1是卖，-1是不进行买卖操作
    for robot_id in range(4):
        # 如果机器人身上没有背东西
        if n_robots[robot_id][1] == 0:
            if sum(n_each_lack_num) == 0:
                each_robot_act[robot_id] = [0, 0, -1]
            elif each_not_carry_robot_toward_bench[robot_id] != -1:
                if each_not_carry_robot_toward_bench[robot_id] == n_robots[robot_id][0]:
                    assumption_bench = pre_carried_robot_tar_bench(robot_id, n_benches[n_robots[robot_id][0]][1])
                    # 只有当时间足够卖掉是才会购买
                    if frame_id <= 8696 or (frame_id > 8696 and n_benches[n_robots[robot_id][0]][1] in [3, 5]):

                        each_robot_act[robot_id][2] = 0  # 购买
                        each_not_carry_robot_toward_bench[robot_id] = -1  # 购买之后，0号机器人就不在抢占这个工作台了
                        each_carry_robot_toward_bench[robot_id] = assumption_bench  # 购买之后立刻指定目标工作台id
                        n_robots[robot_id][1] = n_benches[n_robots[robot_id][0]][1]
                else:
                    # r_instruct_是计算出来的机器人需要执行的命令
                    r_instruct = cal_instruct_1(0,
                                                n_robots[robot_id][7],
                                                n_robots[robot_id][6],
                                                n_benches[each_not_carry_robot_toward_bench[robot_id]][2],
                                                robot_id)
                    each_robot_act[robot_id][0] = r_instruct[0]  # 线速度
                    each_robot_act[robot_id][1] = r_instruct[1]  # 角速度
            else:
                # 首先把所有缺的物品对应的已经生产好了的工作台放到列表中，注意，可能这个列表为空。因为可能这个缺的物品一个都没有生产出来
                l_d_m = []  # 缺少的但是已经做好了的物品对应的工作台，这个工作台的与1号机器人的距离和这个工作台的id
                for k, v in n_done_bench.items():
                    if n_each_lack_num[k] > 0:
                        for bench in v:
                            weight = 1 if n_benches[bench[0]][1] != 7 else 100
                            l_d_m.append([cal_distance(n_robots[robot_id][7][0],
                                                       n_robots[robot_id][7][1],
                                                       bench[1][0],
                                                       bench[1][1]) / weight, bench[0]])

                if l_d_m:
                    l_d_m.sort()  # 按照距离从小到大排序，取第一个没有被其他机器人抢占的工作台，除非这个工作台是123
                    for dis, bench_id in l_d_m:
                        if bench_id not in each_not_carry_robot_toward_bench or n_benches[bench_id][1] in [1, 2, 3]:
                            each_not_carry_robot_toward_bench[robot_id] = bench_id
                            break
                    # 如果有缺的被生产好了，这个机器人要去这个工作台生产的材料需求-1，而且n_each_lack中对这个材料有需求的工作台也要被删除
                    if each_not_carry_robot_toward_bench[robot_id] != -1:
                        n_each_lack_num[n_benches[each_not_carry_robot_toward_bench[robot_id]][1]] -= 1

        else:
            # 机器人身上有，说明一定有工作台需要
            if n_robots[robot_id][0] != -1 and n_robots[robot_id][0] == each_carry_robot_toward_bench[robot_id]:
                each_robot_act[robot_id][2] = 1  # 卖出
                each_carry_robot_toward_bench[robot_id] = -1  # 卖了之后就取消该号机器人对该工作台的抢占
                each_not_carry_robot_toward_bench[robot_id] = -1  # 卖掉之后我就是没有目标的了
                # 卖掉之后这个工作台就不缺这个材料了，极限条件下，0号刚卖1号并解除了自己对这个工作台的抢占，同一帧1号就买同种材料，极有可能指向同一个工作台！
                wait_for_del = 100000  # 初始化再也不写-1了，老是倒着删除
                for ind, v in enumerate(n_each_lack[n_robots[robot_id][1]]):
                    if v[0] == n_robots[robot_id][0]:
                        wait_for_del = ind
                        break
                n_each_lack[n_robots[robot_id][1]].pop(wait_for_del)

            # 如果这个机器人不能与目标工作台交易，则向他移动
            else:
                r_instruct = cal_instruct_1(1,
                                            n_robots[robot_id][7],
                                            n_robots[robot_id][6],
                                            n_benches[each_carry_robot_toward_bench[robot_id]][2],
                                            robot_id)
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


# 给定当前是第几帧，判断现在应该显示几分几秒
def cal_time(frame):
    second, frame = divmod(frame, 50)  # 返回已经进行的秒数和余下来的帧数
    last_second = 180 - second
    last_min, last_second = divmod(last_second, 60)
    return str(last_min) + ' : ' + str(last_second)


def finish():
    sys.stdout.write('OK\n')
    sys.stdout.flush()


# 传入记录所有工作台两两之间距离的列表名，直接修改
def cal_each_bench_distance(each_bench_distance):
    n = len(n_benches)
    for i in range(n):
        for j in range(n):
            if i != j:
                each_bench_distance[i][j] = cal_distance(n_benches[i][2][0], n_benches[i][2][1], n_benches[j][2][0],
                                                         n_benches[j][2][1])


def map_4_main():
    # each_bench_distance[i][j] 表示工作台i到工作台j的距离
    global each_bench_distance
    each_bench_distance = []
    while True:
        # 第一个必须由外面的循环读取，否则不能判断是否已经结束
        # start_time = time.perf_counter()
        line = sys.stdin.readline()
        if not line:
            break
        parts = line.split(' ')
        global frame_id
        frame_id = int(parts[0])
        # 读取信息并得到当前工作台和机器人的状态信息
        global n_benches, n_robots
        n_benches, n_robots = read_status()
        # 更新each_bench_distance，只跟新一次
        if not each_bench_distance:
            each_bench_distance = [[0 for _ in range(len(n_benches))] for _ in range(len(n_benches))]
            cal_each_bench_distance(each_bench_distance)
        # 处理好每一帧需要的4个数据
        global n_type_lack, n_robot_carry, n_each_lack, n_done_bench, n_each_lack_num
        n_type_lack, n_robot_carry, n_each_lack, n_done_bench, n_each_lack_num = init_frame()
        # 这一帧每个机器人应该执行的操作
        #  设置直接忽略123工作台加工时间的地图
        # 根据每一副地图在不同分配方案上的表现具体确定使用哪种分配方案
        n_each_robot_act = task_process_1()
        sys.stdout.write('%d\n' % frame_id)
        for ind, act in enumerate(n_each_robot_act):
            sys.stdout.write('forward %d %f\n' % (ind, act[0]))
            sys.stdout.write('rotate %d %f\n' % (ind, act[1]))
            # 如果使用的是原始的方案，200ms之前就不买了
            if act[2] == 0:
                sys.stdout.write('buy %d \n' % ind)
            elif act[2] == 1:
                sys.stdout.write('sell %d \n' % ind)
        # end_time = time.perf_counter()
        finish()

if __name__ == '__main__':
    map_4_main()
