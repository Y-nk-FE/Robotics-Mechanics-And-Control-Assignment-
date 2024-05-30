# Robotics-Mechanics-And-Control-Assignment-
> [!IMPORTANT]
>
> **注意：如有使用，请遵循开源协议**
>
> 更新时间：2024年5月30日

## 文件说明

XXXXX_VX.doc---对应版本的报告

Solution_VX.m---对应版本的matlab脚本文件[解决方案]

机器人学导论课程作业，其中包括了轨迹规划，PD控制器，无外力和有外力的阻抗控制器（无重力补偿）设计和设计报告
规划和设计使用的语言：Matlab
对于需要迁移到嵌入式设备中或者硬件开发者十分友好，都是较为底层的编程语言，可以很快的迁移到java或者C语言控制的控制器

## 版本说明

### V 0.1

机械臂的物理模型参数如下：

| Joint | m(kg)-质点化质量 | l（m）-连杆长度 |
| :---: | :--------------: | :-------------: |
|   1   |        10        |       100       |
|   2   |        10        |       120       |
|   3   |        10        |        -        |

### V 0.2

此版本对物理参数进行了修正，此外，对应控制器的参数也进行了一定程度的微调



| Joint | m(kg)-质点化质量 | l（m）-连杆长度 |
| :---: | :--------------: | :-------------: |
|   1   |      2.4312      |      0.250      |
|   2   |      3.7860      |      0.250      |
|   3   |      0.5552      |      0.245      |
