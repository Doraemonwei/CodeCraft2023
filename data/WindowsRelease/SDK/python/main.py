#!/bin/bash
import math
import sys
from collections import defaultdict


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
            # 注意bench[2]本来就是个列表[x,y], had_len就是这个工作台已经有材料的数量
            each_lack[m].append([bench[0], bench[2], had_len])
            # 循环能到这里说明不是123，89又没有产品
        if bench[1] not in [8, 9]:
            if bench[5] == 1:
                done_bench[bench[1]].append([bench[0], bench[2]])
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
        if will_carry != -1:
            each_lack_num[n_benches[will_carry][1]] -= 1

    # 如果场上有9工作台，需要对each_lack_num特殊处理
    for i in n_benches:
        if i[1] == 9:
            each_lack_num = [0, 10000, 100000, 10000, 100000, 100000, 100000, 100000, 10000]
            break

    return type_lack, robot_carry, each_lack, done_bench, each_lack_num


each_not_carry_robot_toward_bench = [-1] * 4  # 所有身上没有背着东西的机器人准备去的工作台序号，-1表示没有
each_carry_robot_toward_bench = [-1] * 4  # 所有身上带着东西的机器人准备去的工作台序号，-1表示没有


# 传入机器人id和假设的放在它身上的货品类型，返回这种假设下它应该去的工作台id
def pre_carried_robot_tar_bench(robot_id, assumption_carry):
    if n_robots[robot_id][1] == -1:
        material_type = assumption_carry
    else:
        material_type = n_robots[robot_id][1]
    need_robot_id_type_m_benches = []  # 所有需要0号机器人背上材料的工作台[[add_weight_distance, bench_id]],加权距离，工作台id
    for bench in n_each_lack[material_type]:
        # weight是权重
        weight = bench[2] + 1
        need_robot_id_type_m_benches.append([cal_distance(n_robots[robot_id][7][0],
                                                          n_robots[robot_id][7][1],
                                                          bench[1][0],
                                                          bench[1][1]) / weight,
                                             bench[0]])
    need_robot_id_type_m_benches.sort()  # 按照加权距离进行排序
    # 我的目标点应该是哪个工作台？？？
    assumption_target_bench = -1
    for bench in need_robot_id_type_m_benches:
        flag = False
        for i in range(4):
            if i == robot_id:
                continue
            else:
                if each_carry_robot_toward_bench[i] == bench[1] and n_robots[i][1] == n_robots[robot_id][1] and \
                        n_benches[bench[1]][1] != 9:
                    flag = True
        if flag:
            continue
        else:
            assumption_target_bench = bench[1]
            break

    return assumption_target_bench


def cal_instruct_1(is_carry, robot_loc, robot_angle, bench_loc, robot_id):
    r_x, r_y = robot_loc[0], robot_loc[1]
    b_x, b_y = bench_loc[0], bench_loc[1]

    r2b = [b_x - r_x, b_y - r_y]  # 机器人指向工作台的向量，目标就是把机器人的速度矢量掰过来
    r2b_a = math.atan2(r2b[1], r2b[0])  # 当前机器人与目标工作台向量与x轴正方向的夹角

    distance = math.sqrt((r_x - b_x) ** 2 + (r_y - b_y) ** 2)  # 当前机器人与工作台的距离

    n_line_speed = 6
    n_angle_speed = 0

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

    # 对于地图2的特判
    if which_map[map_mark] in [1, 2, 3, 4]:
        # 地图边界约束
        if (distance <= 5 or (r_x <= 1 or r_x >= 48 or r_y <= 1 or r_y >= 48)) and abs(robot_angle - r2b_a) >= 1.5:
            n_line_speed = 0

    # 对于地图1的特判
    if which_map[map_mark] in [1, 2, 3, 4]:
        # 距离目标约束
        if distance <= 2 and abs(robot_angle - r2b_a) >= 1.5:
            n_line_speed = 0

    # 机器人之间相撞特判，如果两个机器人相聚很近而且二者的速度在两者连线的方向方向相向且较大，当前机器人就顺时针拉满角速度,但是这种方式似乎不太适合很散的图，比如1
    # 当然，感觉这是我程序的问题
    #  首先只考虑距离此机器人最近的机器人
    nearest_robot_id = -1
    nearest_distance = float('inf')
    for r_i in range(4):
        if r_i == robot_id:
            continue
        else:
            t = cal_distance(robot_loc[0], robot_loc[1], n_robots[r_i][7][0], n_robots[r_i][7][1])
            if t < nearest_distance:
                nearest_robot_id = r_i
                nearest_distance = t

    # 如果他们相向而行
    if which_map[map_mark] in [1, 2, 3, 4]:
        if nearest_distance <= 4:
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
                if math.acos(cos_theta_1) + math.acos(cos_theta_2) <= (math.pi / 180) * 90:
                    # 不能简单的都给正角速度
                    if robot_angle >= 0:
                        if n_robots[nearest_robot_id][7][0] < robot_loc[0]:
                            n_angle_speed = -3.4
                        else:
                            n_angle_speed = 3.4
                    else:
                        if n_robots[nearest_robot_id][7][0] < robot_loc[0]:
                            n_angle_speed = 3.4
                        else:
                            n_angle_speed = -3.4

    return [n_line_speed, n_angle_speed]


# 没有老大
def task_process_1():
    # 返回的是每个机器人应该进行的操作，包括线速度，角速度，买卖
    each_robot_act = [[0, 0, -1] for _ in range(4)]  # 0，0表示线速度和角速度，0是买1是卖，-1是不进行买卖操作

    for robot_id in range(4):
        if n_robots[robot_id][1] == 0:
            if sum(n_each_lack_num) == 0:
                each_robot_act[robot_id] = [0, 0, -1]
            elif each_not_carry_robot_toward_bench[robot_id] != -1:
                assumption_bench = pre_carried_robot_tar_bench(robot_id, n_benches[n_robots[robot_id][0]][1])
                pre_time = cal_distance(n_robots[robot_id][7][0], n_robots[robot_id][7][1],
                                        n_benches[assumption_bench][2][0],
                                        n_benches[assumption_bench][2][1]) / 6
                pre_frame = pre_time * 50
                if each_not_carry_robot_toward_bench[robot_id] == n_robots[robot_id][
                    0] and frame_id + pre_frame <= 9000:
                    each_robot_act[robot_id][2] = 0  # 购买
                    each_not_carry_robot_toward_bench[robot_id] = -1  # 购买之后，0号机器人就不在抢占这个工作台了
                else:
                    # r_instruct_0是计算出来的0号机器人需要执行的命令
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
                        # 因为n_done_bench这个字典的值是列表类型，所以需要遍历一个个取
                        for bench in v:
                            # weight
                            weight = 1 if n_benches[bench[0]][1] != 7 else 3
                            l_d_m.append([cal_distance(n_robots[robot_id][7][0],
                                                       n_robots[robot_id][7][1],
                                                       bench[1][0],
                                                       bench[1][1]) / weight, bench[0]])
                # 如果已经有缺少的成品被生产出来了再进行下面的操作，否则就直接不改变默认的速度设定，也就是全0
                if l_d_m:
                    l_d_m.sort()  # 按照距离从小到大排序，取第一个
                    for dis, bench_id in l_d_m:
                        if bench_id not in each_not_carry_robot_toward_bench:
                            each_not_carry_robot_toward_bench[robot_id] = bench_id
                            break
                    # 如果有缺的被身生产好了，这个机器人要去这个工作台生产的材料需求-1
                    if each_not_carry_robot_toward_bench[robot_id] != -1:
                        n_each_lack_num[n_benches[each_not_carry_robot_toward_bench[robot_id]][1]] -= 1

        else:
            # 机器人身上有，说明一定有工作台需要
            if each_carry_robot_toward_bench[robot_id] != -1:
                if n_robots[robot_id][0] == each_carry_robot_toward_bench[robot_id]:
                    each_robot_act[robot_id][2] = 1  # 卖出
                    each_carry_robot_toward_bench[robot_id] = -1  # 卖了之后就取消0号机器人对该工作台的抢占
                # 如果这个机器人不能与目标工作台交易，则向他移动
                else:
                    r_instruct = cal_instruct_1(1,
                                                n_robots[robot_id][7],
                                                n_robots[robot_id][6],
                                                n_benches[each_carry_robot_toward_bench[robot_id]][2],
                                                robot_id)
                    each_robot_act[robot_id][0] = r_instruct[0]  # 线速度
                    each_robot_act[robot_id][1] = r_instruct[1]  # 角速度
            else:
                each_carry_robot_toward_bench[robot_id] = pre_carried_robot_tar_bench(robot_id, n_robots[robot_id][1])
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


# 记录用的是哪一张地图，方便使用对应的分配方案和运动控制
# 地图1： 124235316AA25A431A78712543612423531
# 地图2：123132132AAAA56465478
# 地图3：71233AAAA4564568112233
# 地图4：912312544A5545A45666666455563652415546565A64A566661236
which_map = {'124235316AA25A431A78712543612423531': 1, '123132132AAAA56465478': 2, '71233AAAA4564568112233': 3,
             '912312544A5545A45666666455563652415546565A64A566661236': 4}
if __name__ == '__main__':
    # 读取机器人以及工作台的初始位置信息
    map_mark = read_map_util_ok()
    # 每一次与判题器交互后都要输出ok并flush
    finish()
    while True:
        # 第一个必须由外面的循环读取，否则不能判断是否已经结束
        # start_time = time.perf_counter()
        line = sys.stdin.readline()
        if not line:
            break
        parts = line.split(' ')
        frame_id = int(parts[0])
        # 读取信息并得到当前工作台和机器人的状态信息
        n_benches, n_robots = read_status()
        # 处理好每一帧需要的4个数据
        n_type_lack, n_robot_carry, n_each_lack, n_done_bench, n_each_lack_num = init_frame()
        # 这一帧每个机器人应该执行的操作
        # 根据每一副地图在不同分配方案上的表现具体确定使用哪种分配方案
        if which_map[map_mark] in [1, 2, 3, 4]:
            n_each_robot_act = task_process_1()
        # else:
        #     n_each_robot_act = task_process()
        sys.stdout.write('%d\n' % frame_id)
        for ind, act in enumerate(n_each_robot_act):
            sys.stdout.write('forward %d %f\n' % (ind, act[0]))
            sys.stdout.write('rotate %d %f\n' % (ind, act[1]))
            # 如果最后只剩三秒了，就别买了
            if act[2] == 0 and frame_id <= 9000:
                sys.stdout.write('buy %d \n' % ind)
            elif act[2] == 1:
                sys.stdout.write('sell %d \n' % ind)
        # test_write_file(n_robots[0][5])
        # end_time = time.perf_counter()
        # test_write_file('这一帧使用时间为：{}ms'.format((end_time - start_time) * 1000))


        finish()
