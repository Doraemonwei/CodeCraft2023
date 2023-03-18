# -*- coding: utf-8 -*-
# @Time : 2023/3/16 14:50
# @Author : Lanpangzi
# @File : 猜测种子.py
import json
import subprocess


def get_each_map_score(random_seed):
    each_map_score = [0] * 5
    for i in range(1, 5):
        result = subprocess.run(
            "robot.exe \"python SDK\python\main.py\" -f -s {} -m maps\\{}.txt -r 1.rep".format(random_seed, i),
            shell=True,
            capture_output=True, text=True)
        result = result.stdout.strip()
        user_dict = json.loads(result)
        each_map_score[i] = user_dict['score']
        print('第{}张图的的分是: {}'.format(i, user_dict['score']))
    return each_map_score


if __name__ == '__main__':
    # seed = 1289046
    seed = 233
    print('种子是:{}'.format(seed))
    each_score = get_each_map_score(seed)
    sum_ = 0
    for ind, i in enumerate(each_score):
        if ind >= 1:
            sum_ += i
    print('总分是：{}'.format(sum_))
