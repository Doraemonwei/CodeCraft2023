# CodeCraft2023
华为杯2023

官方baseline 结果是864876，并且似乎没有考虑到碰撞的情况，撞了也不管，就硬撞然后一点点偏移

简单化这一题，可以分为两大问题：

1、多机器人的路径规划，即如何同时控制四个机器人到达各自的目标点

2、最优方案，即每一个机器人应该去哪个点

## 3.11. 新思路

完全按照官方demo推演，并不是像我之前想的，但是仍然是针对每一帧进行的操作，不需要知道之前的状态，也不需要知道之后的状态


## 小问题：

1、工作台8和9不生产物品，要他们干啥？

答：用来收购相应的物品，物品不能在机器人身上凭空卖掉，只能在对应的收这个物品的工作台售卖，机器人只能凭空销毁物品

2、有没有可能有的工作台压根就不会出现？

答：是的，完全有可能

3、判题器给定的工作台和机器人信息都是按照一定的顺序来的，是map里面的顺序吗（从左到右，从上到下）？

答：是按照判题器第一次给你的位置来，第一次判题器发过来的就是初始的状态，里面含有所有的物体以及坐标，并且题目保证每一次的输出顺序一致

4、既然每一次指令发送都会给所有物体的位置信息，那我是不是不用初始化时记录各个物体的位置了？

答：目前来看似乎是这样，初始化时其实只需要大概看一下就行，我甚至感觉不看都行。。。。。

5、我的代码中每一次买卖都是针对当前帧的执行的，并且会改变本地状态的记录，如果有一帧没有对上，岂不是可能导致后面的状态全部出错乱掉？

答：目前来看是的，所以一定要尽量保证不丢帧，后面的完善可以考虑加上实时对比每个机器人身上背的东西，如果与预期不符需要进行修改操作，这些是后话了

6、do_task()中，如果这个交易不能进行怎么办？虽然任务是按照队列顺序进行的，但是执行后面任务的机器人可能比执行前一个任务的机器人先到达，可能这个地方的产品还没有被拿走，
   这种应该如何处理？

答：目前的解决方法是对每一个task使用task_id标记，如果已经完成了就放进done_tasks集合中，

## 思路1（废弃，没必要）
站在上帝的视角看全局，从一开始就规划好了所有的路径，接下来的任务就是如何控制这4个机器人行动的问题

想在本地实时记录所有东西的状态，具体方案为对每一个物体创建一个类，每一帧都更新所有的实例，但是发现好像不太需要，因为所有物体的状态每一帧
0
判题器都会返回给我，我为什么要记录呢？所以就想到了思路二

## 思路2
由于每一帧的情况判题器都会给我，所以我只需要针对这一瞬间的情况给出操作即可，不需要考虑之后的与之前的状态，

对于这种想法比较可行

问题1就是多机器人的路径规划简单版，因为没有障碍，只需要它们自己之间别相互阻挡就行，这是一个独立的部分，网上应该有比较成熟的方案，只要配合判题器稍作修改就行


问题2才是我们能优化的关键，也就是我们要让每一个机器人去哪个工作台的问题。

   目前想到一个最简单的版本：首先每一个机器人都有自己的状态（0，1），0表示这台机器人刚完成任务现在是空闲状态，需要给他分配任务其实就是
   
给一个工作台让他去，1表示这个机器人现在有任务在身，不要管他，让他去执行任务。关键就是没有任务的机器人应该让他去哪个工作台，分配任务的逻辑

如下：

①  观察工作台7有没有成品，有的话直接拿去8或9卖掉(哪个近取哪个)

②  观察工作台7缺不缺原材料，缺的话看4、5、6有没有做好了的，如果有做好了的或正在做都直接去拿，如果到达了这个工作台还没有

做好的话就直接去1、2、3拿这个工作台需要的原材料（这一步可以优化成事先计算好达到目标工作台时有没有做好），回头直接把这个1、2、3卖掉并

拿走4、5、6卖给7，如果到达1、2、3后工作台还没有做好就在那等着


整体的思路就是尽量生成7，但是这可能会浪费大量的时间去走到工作台7，尤其是当7号工作台距离其他的工作台非常远的时候

## 思路二具体化

一共两大部分：

想到一个小点，不是小车去找工作台，而是工作台找小车，所以就盯着456789，只要他们有需求就去找这个需求的根源，让距离这个根源需求最近的小车
去取，这样好像可以规避某些工作台不存在的问题，因为如果一单对这个东西有需求，就一定有工作台可以出售，极端状态如果123都不存在的话，根源需求永远不会得到满足，
小车永远不会动

宏观来看，每一个小车除了在完成交易的瞬间没有任务外，其他所有时刻身上应该都是背着任务（目标点）的，因此每一帧结束时其实小车都有任务在身，

换句话说小车一直有目标要去

1、给定当前小车的位置信息运动状态和目标点，给我这一时刻需要对这个小车发出的指令

2、已知当前所有工作台的状态，如何给小车分配任务，也就是说如何指定每一个小车的目标工作台


 
## 代码bug修改
1、图3运行到剩余24-22s的时候会有三个机器人停止不动，但是很明显场上缺了好几个23，而且也有23已经做好了的工作台，但是他们都没有去，为什么？
已解决

2、目前每一帧的运算速度是0.15ms左右，完全足够

3、暂时计划加入dwa的路径规划


4、当场上缺少多种物品而且这些缺的都已经生产好了的情况下，优先去最高级的点，而不是最近的！

5、机器人会远离目标点，速度负的打分反而高，因为归一化值是负的！
直接把最小速度设置为0

6、机器人在靠近目标是会减速
因为当靠近目标的时候，距离太快的在预判的时间范围内会超过目标点，使得角度评价分数变得特别低，所以角度评价
时，需要考虑是否已经距离目标点足够近，如果足够近就直接给一个很高的分

