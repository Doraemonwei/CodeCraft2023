# -*- coding: utf-8 -*-
# @Time : 2023/3/16 14:50
# @Author : Lanpangzi
# @File : 猜测种子.py
import json
import subprocess

tar_score = [0, 837801, 693447, 564698, 693310]


def get_each_map_score(random_seed):
    each_map_score = [0] * 5
    for i in range(1, 5):
        result = subprocess.run(
            "robot.exe \"python SDK\python\main.py\" -f -s {} -m maps\\{}.txt -r 1.rep".format(random_seed, i), shell=True,
            capture_output=True, text=True)
        result = result.stdout.strip()
        user_dict = json.loads(result)
        each_map_score[i] = user_dict['score']
        if each_map_score[i] != tar_score[i]:
            return [], i
    return each_map_score, 5


def test_write_seed(a):
    with open("target_seed.txt", "a", encoding='utf-8') as variable_name:
        variable_name.write(str(a) + '\n')


for i in range(5253, 9999):
    each_score, n = get_each_map_score(i)
    if each_score:
        print('!!!!!！！！！！！！！！！！!!!已经找到了，种子是：{}！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！'.format(i))
        test_write_seed('种子是：{}'.format('i"'))
        break
    else:
        print('{}  不对'.format(i))
