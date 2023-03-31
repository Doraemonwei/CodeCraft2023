#!/bin/bash
import heapq
import math
import sys
from collections import defaultdict
import numpy as np


class PIDController:
    def __init__(self, Kp, Ki, Kd, max_output, max_integral):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.max_output = max_output
        self.integral = 0
        self.previous_error = 0

    def control(self, error, dt):
        self.integral += error * dt
        self.integral = max(min(self.integral, self.max_integral), -self.max_integral)
        derivative = (error - self.previous_error) / dt if dt > 0 else 0
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        output = max(min(output, self.max_output), -self.max_output)
        self.previous_error = error
        return output

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
            each_lack_num[7] = 1000
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
    for bench in n_each_lack[material_type]:
        # weight是权重
        weight = 2 ** bench[2]
        need_robot_id_type_m_benches.append([cal_distance(n_robots[robot_id][7][0],
                                                          n_robots[robot_id][7][1],
                                                          bench[1][0],
                                                          bench[1][1]) / weight,
                                             bench[0]])
    need_robot_id_type_m_benches.sort()  # 按照加权距离进行排序
    # test_write_file(need_robot_id_type_m_benches)
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
                        n_benches[bench[1]][1] not in [9, 8]:
                    flag = True
                    break
        if flag:
            continue
        else:
            asumption_target_bench = bench[1]
            break

    # test_write_file('需要{}类材料的工作台有：{}，{}选择的工作台是：{}'.format(material_type,need_robot_id_type_m_benches, robot_id,asumption_target_bench))
    return asumption_target_bench


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


# 通过pid来控制机器人运动，只需要传进机器人的id即可
def PID_instruct(robot_id):
    # 每一次控制是20ms
    dt = 0.02
    if each_robot_step[robot_id] < len(each_robot_path[robot_id]):
        x_target, y_target = each_robot_path[robot_id][each_robot_step[robot_id]]
        x_target, y_target = cell2loc(x_target, y_target)
        n_x, n_y = n_robots[robot_id][7][0], n_robots[robot_id][7][1]
        distance_error = math.sqrt(
            (x_target - n_x) ** 2 + (y_target - n_y) ** 2)
        if distance_error < 0.05:
            each_robot_step[robot_id] += 1
            return 0, 0
        angle_to_target = math.atan2(y_target - n_y, x_target - n_x)
        angle_error = angle_to_target - n_robots[robot_id][6]
        if angle_error > math.pi:
            angle_error -= 2 * math.pi
        elif angle_error < -math.pi:
            angle_error += 2 * math.pi

        v = each_robot_pid[robot_id][0].control(distance_error, dt)
        w = each_robot_pid[robot_id][1].control(angle_error, dt)
        test_write_file(
            '{}号机器人目标是：{},pid给定的v:{},w:{}'.format(robot_id, each_robot_path[robot_id][each_robot_step[robot_id]], v, w))
        w = w % (2 * np.pi)
        if w > np.pi:
            w -= 2 * np.pi
        return v, w
    else:
        return 0, 0


# 将四个地图的运动控制完全分开
def cal_instruct_1(is_carry, robot_loc, robot_angle, bench_loc, robot_id):
    # return map_1_instruct(is_carry, robot_loc, robot_angle, bench_loc, robot_id)
    return PID_instruct(robot_id)


each_not_carry_robot_toward_bench = [-1] * 4  # 所有身上没有背着东西的机器人准备去的工作台序号，-1表示没有
each_carry_robot_toward_bench = [-1] * 4  # 所有身上带着东西的机器人准备去的工作台序号，-1表示没有


# 空车时直接拿  机器人到成品距离+将成品运送到目标工作台距离最近的那个
#   所有物品的权重一样
def task_process_1():
    # 返回的是每个机器人应该进行的操作，包括线速度，角速度，买卖
    each_robot_act = [[0, 0, -1] for _ in range(4)]  # 0，0表示线速度和角速度，0是买1是卖，-1是不进行买卖操作
    for robot_id in range(4):
        # 如果机器人身上没有背东西
        if n_robots[robot_id][1] == 0:
            if each_not_carry_robot_toward_bench[robot_id] != -1:
                if each_not_carry_robot_toward_bench[robot_id] == n_robots[robot_id][0]:
                    assumption_bench = pre_carried_robot_tar_bench(robot_id, n_benches[n_robots[robot_id][0]][1])
                    # test_write_file(assumption_bench)
                    pre_time = cal_distance(n_robots[robot_id][7][0], n_robots[robot_id][7][1],
                                            n_benches[assumption_bench][2][0],
                                            n_benches[assumption_bench][2][1]) / 6
                    pre_frame = pre_time * 50
                    # 只有当时间足够卖掉是才会购买
                    if frame_id + pre_frame <= 9000:
                        each_robot_act[robot_id][2] = 0  # 购买
                        each_not_carry_robot_toward_bench[robot_id] = -1  # 购买之后，0号机器人就不在抢占这个工作台了
                        each_carry_robot_toward_bench[robot_id] = assumption_bench  # 购买之后立刻指定目标工作台id
                        n_robots[robot_id][1] = n_benches[n_robots[robot_id][0]][1]
                        r_x, r_y = n_robots[robot_id][7]
                        b_x, b_y = n_benches[assumption_bench][2]
                        r_x_c, r_y_c = loc2cell(r_x, r_y)
                        b_x_c, b_y_c = loc2cell(b_x, b_y)
                        # test_write_file('坐标为：{}，转换后cell为')
                        path = get_path((r_x_c, r_y_c), (b_x_c, b_y_c))
                        each_robot_path[robot_id] = path
                        each_robot_step[robot_id] = 0
                        # 将pid重新置零
                        each_robot_pid[robot_id][0].last_error = 0
                        each_robot_pid[robot_id][1].integral = 0


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
                        # 因为n_done_bench这个字典的值是列表类型，所以需要遍历一个个取
                        for bench in v:
                            weight = [0, 1, 1, 1, 1, 1, 6, 8]
                            weight = weight[n_benches[bench[0]][1]]
                            l_d_m.append([cal_distance(n_robots[robot_id][7][0],
                                                       n_robots[robot_id][7][1],
                                                       bench[1][0],
                                                       bench[1][1]) / weight, bench[0]])
                # 如果已经有缺少的成品被生产出来了再进行下面的操作，否则就直接不改变默认的速度设定，也就是全0
                if l_d_m:
                    l_d_m.sort()  # 按照距离从小到大排序，取第一个没有被其他机器人抢占的工作台，除非这个工作台是123
                    for dis, bench_id in l_d_m:
                        if bench_id not in each_not_carry_robot_toward_bench or n_benches[bench_id][1] in [1, 2, 3]:
                            # if bench_id not in each_not_carry_robot_toward_bench:
                            each_not_carry_robot_toward_bench[robot_id] = bench_id
                            break
                    if each_not_carry_robot_toward_bench[robot_id] != -1:
                        r_x, r_y = n_robots[robot_id][7][0], n_robots[robot_id][7][1]
                        b_x, b_y = n_benches[each_not_carry_robot_toward_bench[robot_id]][2]
                        r_x_c, r_y_c = loc2cell(r_x, r_y)
                        b_x_c, b_y_c = loc2cell(b_x, b_y)
                        path = get_path((r_x_c, r_y_c), (b_x_c, b_y_c))
                        each_robot_path[robot_id] = path
                        each_robot_step[robot_id] = 0
                        # 将pid重新置零
                        each_robot_pid[robot_id][0].last_error = 0
                        each_robot_pid[robot_id][1].integral = 0
                        test_write_file('{}号机器人的路径为：{}'.format(robot_id, path))

                    # 如果有缺的被生产好了，这个机器人要去这个工作台生产的材料需求-1
                    if each_not_carry_robot_toward_bench[robot_id] != -1 and \
                            n_benches[each_not_carry_robot_toward_bench[robot_id]][1] not in [8, 9]:
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


# 查看机器人的位置以及朝向信息，并查看这一帧给这个机器人的操作指令
def view_robot_status(start, end, n_each_robot_act):
    if start <= frame_id <= end:
        test_write_file('frame_id:{}'.format(frame_id))
        for i in range(4):
            test_write_file('机器人id:{} \n信息:{}'.format(i, n_robots[i]))
            test_write_file('给这个机器人的指令是：{}'.format(n_each_robot_act[i]))


# 路线规划
class Env:
    def __init__(self, obs):
        self.x_range = 100  # size of background
        self.y_range = 100
        self.motions = [(-1, 0), (-1, 1), (0, 1), (1, 1),
                        (1, 0), (1, -1), (0, -1), (-1, -1)]
        self.obs = obs

    def update_obs(self, obs):
        self.obs = obs
        return obs


class AStar:
    """AStar set the cost + heuristics as the priority
    """

    def __init__(self, s_start, s_goal, heuristic_type, env):
        self.s_start = s_start
        self.s_goal = s_goal
        self.heuristic_type = heuristic_type

        self.Env = env  # class Env

        self.u_set = self.Env.motions  # feasible input set
        self.obs = self.Env.obs  # position of obstacles

        self.OPEN = []  # priority queue / OPEN set
        self.CLOSED = []  # CLOSED set / VISITED order
        self.PARENT = dict()  # recorded parent
        self.g = dict()  # cost to come

    def searching(self):
        """
        A_star Searching.
        :return: path, visited order
        """

        self.PARENT[self.s_start] = self.s_start
        self.g[self.s_start] = 0
        self.g[self.s_goal] = math.inf
        heapq.heappush(self.OPEN,
                       (self.f_value(self.s_start), self.s_start))
        num = 0
        while self.OPEN:

            _, s = heapq.heappop(self.OPEN)
            self.CLOSED.append(s)
            if s == self.s_goal:  # stop condition
                break
            for s_n in self.get_neighbor(s):
                new_cost = self.g[s] + self.cost(s, s_n)
                if s_n not in self.g:
                    self.g[s_n] = math.inf
                if new_cost < self.g[s_n]:  # conditions for updating Cost
                    self.g[s_n] = new_cost
                    self.PARENT[s_n] = s
                    heapq.heappush(self.OPEN, (self.f_value(s_n), s_n))
            num += 1

        return self.extract_path(self.PARENT), self.CLOSED

    def searching_repeated_astar(self, e):
        """
        repeated A*.
        :param e: weight of A*
        :return: path and visited order
        """

        path, visited = [], []

        while e >= 1:
            p_k, v_k = self.repeated_searching(self.s_start, self.s_goal, e)
            path.append(p_k)
            visited.append(v_k)
            e -= 0.5

        return path, visited

    def repeated_searching(self, s_start, s_goal, e):
        """
        run A* with weight e.
        :param s_start: starting state
        :param s_goal: goal state
        :param e: weight of a*
        :return: path and visited order.
        """

        g = {s_start: 0, s_goal: float("inf")}
        PARENT = {s_start: s_start}
        OPEN = []
        CLOSED = []
        heapq.heappush(OPEN,
                       (g[s_start] + e * self.heuristic(s_start), s_start))

        while OPEN:
            _, s = heapq.heappop(OPEN)
            CLOSED.append(s)

            if s == s_goal:
                break

            for s_n in self.get_neighbor(s):
                new_cost = g[s] + self.cost(s, s_n)

                if s_n not in g:
                    g[s_n] = math.inf

                if new_cost < g[s_n]:  # conditions for updating Cost
                    g[s_n] = new_cost
                    PARENT[s_n] = s
                    heapq.heappush(OPEN, (g[s_n] + e * self.heuristic(s_n), s_n))

        return self.extract_path(PARENT), CLOSED

    def get_neighbor(self, s):
        """
        find neighbors of state s that not in obstacles.
        :param s: state
        :return: neighbors
        """

        return [(s[0] + u[0], s[1] + u[1]) for u in self.u_set]

    def cost(self, s_start, s_goal):
        """
        Calculate Cost for this motion
        :param s_start: starting node
        :param s_goal: end node
        :return:  Cost for this motion
        :note: Cost function could be more complicate!
        """

        if self.is_collision(s_start, s_goal):
            return math.inf

        return math.hypot(s_goal[0] - s_start[0], s_goal[1] - s_start[1])

    def is_collision(self, s_start, s_end):
        """
        check if the line segment (s_start, s_end) is collision.
        :param s_start: start node
        :param s_end: end node
        :return: True: is collision / False: not collision
        """

        if s_start in self.obs or s_end in self.obs:
            # test_write_file('起点或终点在障碍物中1')
            return True

        if s_start[0] != s_end[0] and s_start[1] != s_end[1]:
            if s_end[0] - s_start[0] == s_start[1] - s_end[1]:
                s1 = (min(s_start[0], s_end[0]), min(s_start[1], s_end[1]))
                s2 = (max(s_start[0], s_end[0]), max(s_start[1], s_end[1]))
            else:
                s1 = (min(s_start[0], s_end[0]), max(s_start[1], s_end[1]))
                s2 = (max(s_start[0], s_end[0]), min(s_start[1], s_end[1]))

            if s1 in self.obs or s2 in self.obs:
                # test_write_file('起点或终点在障碍物中2')
                return True

        return False

    def f_value(self, s):
        """
        f = g + h. (g: Cost to come, h: heuristic value)
        :param s: current state
        :return: f
        """

        return self.g[s] + self.heuristic(s)

    def extract_path(self, PARENT):
        """
        Extract the path based on the PARENT set.
        :return: The planning path
        """
        path = [self.s_goal]
        s = self.s_goal
        # test_write_file('开始extract_path,OARENT:{}'.format(PARENT))

        while True:
            s = PARENT[s]
            path.append(s)

            if s == self.s_start:
                break

        return list(path)

    def heuristic(self, s):
        """
        Calculate heuristic.
        :param s: current node (state)
        :return: heuristic function value
        """

        heuristic_type = self.heuristic_type  # heuristic type
        goal = self.s_goal  # goal node

        if heuristic_type == "manhattan":
            return abs(goal[0] - s[0]) + abs(goal[1] - s[1])
        else:
            return math.hypot(goal[0] - s[0], goal[1] - s[1])


def get_path(s_start, s_goal):
    """
    :param s_start: 开始点方格
    :param s_goal: 目标点方格
    :return: 包含初始与目标方格的路径列表
    """
    m1_env = Env(obs)
    s_start = s_start
    s_goal = s_goal
    # test_write_file('开始路径规划，起点为{}，终点为：{}，障碍物为：{}'.format(s_start, s_goal, m1_env.obs))
    astar = AStar(s_start, s_goal, "euclidean", env=m1_env)
    path, visited = astar.searching()
    return path[::-1]


# 每一个机器人被规划出来的路径
each_robot_path = [[], [], [], []]
# step就是规划出来的路径的索引，当路径生成后-1将变成0
each_robot_step = [-1, -1, -1, -1]
# 每一个机器人对应的pid控制器，每一个机器人有一对，分别控制线速度v和角速度w
each_robot_pid = [[PIDController(1, 0, 0), PIDController(0.1, 0.01, 0.01)] for _ in range(4)]


def map_1_main(m1_obs):
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
        #  设置直接忽略123工作台加工时间的地图
        # 根据每一副地图在不同分配方案上的表现具体确定使用哪种分配方案
        n_each_robot_act = task_process_1()
        # test_write_file('n_each_act:{}'.format(n_each_robot_act))
        # end_time = time.perf_counter()
        # test_write_file('{}帧使用时间为：{}ms'.format(frame_id, (end_time - start_time) * 1000))
        sys.stdout.write('%d\n' % frame_id)
        for ind, act in enumerate(n_each_robot_act):
            sys.stdout.write('forward %d %f\n' % (ind, act[0]))
            sys.stdout.write('rotate %d %f\n' % (ind, act[1]))
            # 如果使用的是原始的方案，200ms之前就不买了
            if act[2] == 0:
                sys.stdout.write('buy %d \n' % ind)
            elif act[2] == 1:
                sys.stdout.write('sell %d \n' % ind)

        # view_robot_status(2200, 2250, n_each_robot_act)

        finish()


if __name__ == '__main__':
    map_1_main()
