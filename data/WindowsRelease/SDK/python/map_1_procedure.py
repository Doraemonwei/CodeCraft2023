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
def lack_which_material(bench_type, material_status):
    each_bench_materials = {4: {1, 2}, 5: {1, 3}, 6: {2, 3}, 7: {4, 5, 6}, 8: {7},
                            9: {1, 2, 3, 4, 5, 6, 7}}  # 方便判断缺不缺材料，对图一而言注意9号工作台只缺7
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
            each_lack_num[7] = 1000
            break
    return type_lack, robot_carry, each_lack, done_bench, each_lack_num


def cell2loc(cell_x, cell_y):
    """
    :param x: 方格行数
    :param y: 方格列数
    :return: 方格的中心坐标

    """
    x = 0.25 + 0.5 * cell_x
    y = 0.25 + 0.5 * cell_y
    return x, y


def loc2cell(x, y):
    """
    :param x: 真实的横坐标
    :param y: 真实的纵坐标
    :return: （row,column）,方格坐标

    """
    cell_x = math.ceil(2 * x) - 1
    cell_y = math.ceil(2 * y) - 1
    return cell_x, cell_y


# 使用原来的的最简单的控制小车的方式
def traditional_instruct(robot_id):
    test_write_file("{}号小车的整个轨迹为：{}".format(robot_id, each_robot_path[robot_id]))
    if each_robot_step[robot_id] < len(each_robot_path[robot_id]):
        x_target, y_target = each_robot_path[robot_id][each_robot_step[robot_id]]
        # 将格子转换成实际的坐标
        x_target, y_target = cell2loc(x_target, y_target)
        # 现在机器人的实际坐标
        n_x, n_y = n_robots[robot_id][7][0], n_robots[robot_id][7][1]
        # test_write_file('3号工作台的信息：{}'.format(n_benches[3]))
        test_write_file(
            '当前小车的坐标为：{}，目标格子：{}，格子实际坐标：{}'.format([n_x, n_y], each_robot_path[robot_id][each_robot_step[robot_id]],
                                                   [x_target, y_target]))
        distance_error = math.sqrt(
            (x_target - n_x) ** 2 + (y_target - n_y) ** 2)
        if distance_error < 0.3:
            each_robot_step[robot_id] += 1
            return 0, 0
        # 当前机器人与目标工作台向量与x轴正方向的夹角
        r2b_a = math.atan2(y_target - n_y, x_target - n_x)
        robot_angle = n_robots[robot_id][6]
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
        # 距离目标距离对线速度的约束，防止打转
        if distance_error <= 4.5:
            if abs(robot_angle - r2b_a) >= 1.56:
                line_speed = 1.1
            if distance_error <= 1:
                if abs(robot_angle - r2b_a) >= 1.56:
                    line_speed = 0
        return line_speed, angle_speed

    else:
        return 0, 0


# 将四个地图的运动控制完全分开
def cal_instruct_1(is_carry, robot_loc, robot_angle, bench_loc, robot_id):
    # return map_1_instruct(is_carry, robot_loc, robot_angle, bench_loc, robot_id)
    # return PID_instruct(robot_id)
    return traditional_instruct(robot_id)


each_not_carry_robot_toward_bench = [-1] * 4  # 所有身上没有背着东西的机器人准备去的工作台序号，-1表示没有
each_carry_robot_toward_bench = [-1] * 4  # 所有身上带着东西的机器人准备去的工作台序号，-1表示没有

# 每一个机器人被规划出来的路径
each_robot_path = [[], [], [], []]
# step就是规划出来的路径的索引，当路径生成后-1将变成0
each_robot_step = [-1, -1, -1, -1]


# 空车时直接拿  机器人到成品距离+将成品运送到目标工作台距离最近的那个
#   所有物品的权重一样
def task_process_1():
    # 返回的是每个机器人应该进行的操作，包括线速度，角速度，买卖
    each_robot_act = [[0, 0, -1] for _ in range(4)]  # 0，0表示线速度和角速度，0是买1是卖，-1是不进行买卖操作
    for robot_id in range(4):
        # 如果机器人身上没有背东西
        if n_robots[robot_id][1] == 0:
            pass

        else:
            pass
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


# 查看机器人的位置以及朝向信息，并查看这一帧给这个机器人的操作指令
def view_robot_status(start, end, n_each_robot_act):
    if start <= frame_id <= end:
        test_write_file('frame_id:{}'.format(frame_id))
        for i in range(4):
            test_write_file('机器人id:{} \n信息:{}'.format(i, n_robots[i]))
            test_write_file('给这个机器人的指令是：{}'.format(n_each_robot_act[i]))


def map_1_main():
    # 路径规划相关参数
    global obs
    obs = m1_obs
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
    map_1_main()
