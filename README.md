# TEB_Planning

# 参考文献：

[csdn论文翻译《Trajectory modification considering dynamic constraints of autonomous robots》考虑动力学约束的自主机器人轨迹修正](https://blog.csdn.net/qq_43867541/article/details/108777620)

[costmap和teb的介绍-比较有意思](https://www.leiphone.com/category/transportation/0TCtaBOIcFOIBN69.html)

[mooc teb算法介绍](https://www.icourse163.org/learn/ZJU-1206447854?tid=1465429550#/learn/content?type=detail&id=1244596402),对应的ppt:[PPT](Mooc_ElasticBand.pdf)

# teb_local_planner官方Summary翻译

参考链接：[teb_local_planner官方summary](http://wiki.ros.org/teb_local_planner)

## Package Summary

teb_local_planner package是2D导航的base_local_planner的一个插件，称为Timed Elastic Band，在轨迹执行时间、和障碍物的分离、运行时符合的运动学约束进行局部优化机器人轨迹。

- 开发者状态：维护

- 开发者：Christoph Rösmann <christoph.roesmann AT tu-dortmund DOT de>
- 作者：Christoph Rösmann <christoph.roesmann AT tu-dortmund DOT de>
- 许可证：BSD
- 来源：git [https://github.com/rst-tu-dortmund/teb_local_planner.git](https://github.com/rst-tu-dortmund/teb_local_planner)（分支：melodic-devel）

## 1、Overview

***Update:\* 两个关于添加了动态障碍物的规划教程 ([click here](http://wiki.ros.org/teb_local_planner/Tutorials))**

该包实现了一个用于移动机器人导航和控制的在线最优局部轨迹规划器，作为 ROS[导航](http://wiki.ros.org/navigation)包的插件。全局规划器生成的初始轨迹在运行时优化，最小化轨迹执行时间（时间最优目标）， 与障碍物分离并遵守运动动力学约束，例如满足最大速度和加速度。

当前的实现符合非完整机器人（差动驱动和类汽车机器人）的运动学。自 Kinetic 以来，包括对完整机器人的支持。

通过解决稀疏标量多目标优化问题，可以有效地获得最佳轨迹。用户可以为优化问题提供权重，以便在目标冲突的情况下指定行为。

称为“Timed-Elastic-Band”的方法在下面两篇论文中有介绍：

- C. Rösmann, W. Feiten, T. Wösch, F. Hoffmann and T. Bertram: Trajectory modification considering dynamic constraints of autonomous robots. Proc. 7th German Conference on Robotics, Germany, Munich, 2012, pp 74–79.[中文翻译](papers/Trajectory_modification_considering_dynamic_constraints_of_autonomous_robots_2012.md), [英文原文](papers/Trajectory_modification_considering_dynamic_constraints_of_autonomous_robots_2012.pdf)
- C. Rösmann, W. Feiten, T. Wösch, F. Hoffmann and T. Bertram: Efficient trajectory optimization using a sparse model. Proc. IEEE European Conference on Mobile Robots, Spain, Barcelona, 2013, pp. 138–143.[英文原文](papers/Efficient_trajectory_optimization_using_a_sparse_model.pdf)

由于诸如 Timed-Elastic-Band 之类的局部规划器经常陷入局部最优轨迹，因为它们无法穿越障碍物，因此实施了扩展。并行优化独特拓扑的可接受轨迹的子集。局部规划器能够在候选集合中切换到当前的全局最优轨迹。通过利用同源/同伦类的概念获得独特的拓扑。以下论文描述了该方法

- C. Rösmann, F. Hoffmann and T. Bertram: Integrated online trajectory planning and optimization in distinctive topologies, Robotics and Autonomous Systems, Vol. 88, 2017, pp. 142–153. [英文原文](papers/Integrated_online_trajectory_planning_and_optimization_in_distinctive_topologies.pdf)
- C. Rösmann, F. Hoffmann and T. Bertram: Planning of Multiple Robot Trajectories in Distinctive Topologies, Proc. IEEE European Conference on Mobile Robots, UK, Lincoln, Sept. 2015 [英文原文](papers/Planning_of_multiple_robot_trajectories_in_distinctive_topologies.pdf)

对类似汽车的机器人的扩展描述在：

- C. Rösmann, F. Hoffmann and T. Bertram: Kinodynamic Trajectory Optimization and Control for Car-Like Robots, IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), Vancouver, BC, Canada, Sept. 2017. [英文原文](papers/Kinodynamic_Trajectory_Optimization_and_Control_for_Car_Like_Robots.pdf)



通过完成[教程](http://wiki.ros.org/teb_local_planner/Tutorials)部分中的[教程](http://wiki.ros.org/teb_local_planner/Tutorials)开始。

## 2、Video

[video1](https://www.youtube.com/watch?v=o5wnRCzdUMo&feature=emb_rel_end)中介绍了该软件包的功能，并展示了模拟和真实机器人情况的示例

[video2](https://www.youtube.com/watch?v=tab6cdW1kp4&feature=emb_rel_end)中是在0.2版本中引入的功能（支持类汽车机器人和costmap转换）

## 3、NodeAPI

### 3.1、Topics

#### 3.1.1、发布的topics

`~<name>/global_plan` ([nav_msgs/Path](http://docs.ros.org/en/api/nav_msgs/html/msg/Path.html))：全局坐标系下的路径，主要用于可视化。

`~<name>/local_plan` ([nav_msgs/Path](http://docs.ros.org/en/api/nav_msgs/html/msg/Path.html))：局部坐标系下的路径，主要用于可视化。

`~<name>/teb_poses` ([geometry_msgs/PoseArray](http://docs.ros.org/en/api/geometry_msgs/html/msg/PoseArray.html)) ：当前局部规划的离散姿态列表(SE2)，主要用于可视化。

`~<name>/teb_markers` ([visualization_msgs/Marker](http://docs.ros.org/en/api/visualization_msgs/html/msg/Marker.html)) ：teb_local_planner 通过具有不同命名空间的标记提供规划场景的附加信息。

命名空间*[PointObstacles](http://wiki.ros.org/PointObstacles)*和*[PolyObstacles](http://wiki.ros.org/PolyObstacles)*：可视化当前在优化过程中考虑的所有点和多边形障碍物。

命名空间*[TebContainer](http://wiki.ros.org/TebContainer)*：可视化所有找到的和优化的位于替代拓扑中的轨迹（仅当启用并行规划时）。发布了更多信息，例如优化足迹模型。

`~<name>/teb_feedback` ([teb_local_planner/FeedbackMsg](http://docs.ros.org/en/api/teb_local_planner/html/msg/FeedbackMsg.html)) ：反馈消息包含规划轨迹，包括速度剖面和时间信息以及障碍物列表。主要用于评估和调试。必须启用参数`~<name>/publish_feedback`。

#### 3.1.2、订阅的topics

`~<name>/odom` ([nav_msgs/Odometry](http://docs.ros.org/en/api/nav_msgs/html/msg/Odometry.html)) ：为局部规划器提供机器人当前速度的里程计信息。通过重新映射或更改参数`~<name>/odom_topic 来`更改此topic。

`~<name>/obstacles` ([costmap_converter/ObstacleArrayMsg](http://docs.ros.org/en/api/costmap_converter/html/msg/ObstacleArrayMsg.html)) ：将自定义障碍物提供为点状、线状或多边形状的障碍物（或代替costmap障碍物）。

`~<name>/via_points` ([nav_msgs/Path](http://docs.ros.org/en/api/nav_msgs/html/msg/Path.html)) ：提供自定义的via-points（你需要将`~<name>/global_plan_viapoint_sep`设置为零或负数）

### 3.2、Parameters

teb_local_planner 包允许用户设置[参数](http://wiki.ros.org/Parameters)来自定义behavior。这些参数分为几类：机器人配置（obot configuration）、目标容差（goal tolerance）、轨迹配置（trajectory configuration）、障碍物（obstacles）、优化（optimization）、独特拓扑中的规划（planning in distinctive topologies）和杂项参数（miscellaneous parameters）。其中一些被选中以符合[base_local_planner](http://wiki.ros.org/base_local_planner)。许多（但不是全部）参数可以在运行时使用[rqt_reconfigure](http://wiki.ros.org/rqt_reconfigure)进行修改。

#### 3.2.1、Robot Configure Parameters

| 参数名                   | 默认值 | 参数含义                                                     |
| ------------------------ | :----: | :----------------------------------------------------------- |
| acc_lim_x                |  0.5   | 机器人的最大平移加速度(m/s^2)                                |
| acc_lim_theta            |  0.5   | 机器人的最大角加速度(rad/s^2)                                |
| max_vel_x                |  0.4   | 机器人的最大平移速度(m/s)                                    |
| max_vel_x_backwards      |  0.2   | 机器人向后行驶的最大绝对平移速度(m/s),见优化参数weight_kinematics_forward_drive |
| max_vel_theta            |  0.3   | 机器人的最大角速度(rad/s)                                    |
| min_turning_radius       |  0.0   | 汽车机器人的最小转弯半径(对于差速驱动机器人设置为0)          |
| wheelbase                |  1.0   | 后轴与前轴之间的距离。对于`后轮`机器人，该值可能为负（仅当`~<name>/cmd_angle_instead_rotvel`设置为`true`时才需要） |
| cmd_angle_instead_rotvel | false  | 用相应的转向角[-pi/2,pi/2]代替指令速度消息中的旋转速度。     |

以下参数仅与完整机器人相关：**New in ROS kinetic**

**请注意**，显着减少`~<name>/weight_kinematics_nh`以调整顺应纵向运动和非顺应横向运动（扫射）之间的权衡。

| 参数名    | 默认值 | 参数含义                                    |
| --------- | ------ | ------------------------------------------- |
| max_vel_y | 0.0    | 机器人的最大扫射速度（非完整机器人应为0！） |
| acc_lim_y | 0.5    | 机器人最大扫射加速度                        |

以下参数与用于优化的足迹模型相关（参见[教程避障和机器人足迹模型](http://wiki.ros.org/teb_local_planner/Tutorials/Obstacle Avoidance and Robot Footprint Model)）。**New in version 0.3**

| 参数名                       | 默认值                      | 参数含义                                                     |
| ---------------------------- | --------------------------- | ------------------------------------------------------------ |
| footprint_model/type         | "point"                     | 指定用于优化的机器人足迹模型类型。不同的类型是 "point", "circular", "line", "two_circles" and "polygon."。模型的类型会显着影响所需的计算时间。 |
| footprint_model/radius       | 0.2                         | 此参数仅与“circular”类型相关。它包含圆的半径。圆心位于机器人的旋转轴上。 |
| footprint_model/line_start   | [-0.3, 0.0]                 | 此参数仅与“line”类型相关。它包含线段的起始坐标               |
| footprint_model/line_end     | [0.3, 0.0]                  | 此参数仅与“line”类型相关。它包含线段的结束坐标。             |
| footprint_model/front_offset | 0.2                         | 此参数仅与“two_circles”类型相关。它描述了前圆的中心沿机器人的 x 轴移动了多少。假设机器人的旋转轴位于 [0,0]。 |
| footprint_model/front_radius | 0.2                         | 此参数仅与“two_circles”类型相关。它包含前圆的半径。          |
| footprint_model/rear_offset  | 0.2                         | 此参数仅与“two_circles”类型相关。它描述了后圆的中心沿机器人的负 x 轴移动了多少。假设机器人的旋转轴位于 [0,0] |
| footprint_model/rear_radius  | 0.2                         | 此参数仅与“two_circles”类型相关。它包含后圆的半径。          |
| footprint_model/vertices     | [ [0.25,-0.05], [...], ...] | 此参数仅与“polygon”类型相关。它包含多边形顶点列表（每个顶点为 2d 坐标）。多边形始终是封闭的：不要在末尾重复第一个顶点。 |
| is_footprint_dynamic         | false                       | 如果为真，则在检查轨迹可行性之前更新足迹                     |

#### 3.2.2、Goal Tolerance(容差) Parameters

| 参数名             | 默认值 | 参数含义                                         |
| ------------------ | ------ | ------------------------------------------------ |
| xy_goal_tolerance  | 0.2    | 允许到目标位置的直线距离误差（m）                |
| yaw_goal_tolerance | 0.2    | 允许的最终方向误差(rad)                          |
| free_goal_vel      | false  | 去除目标速度约束，使机器人能够以最大速度达到目标 |

#### 3.2.3、Trajectory配置参数

| 参数名                            | 默认值 | 参数含义                                                     |
| --------------------------------- | ------ | ------------------------------------------------------------ |
| dt_ref                            | 0.3    | 轨迹的期望时间分辨率（不固定为`dt_ref，`因为时间分辨率是优化的一部分，但如果违反*dt_ref +-dt_hysteresis，*将在迭代之间调整轨迹大小 |
| dt_hysteresis                     | 0.1    | 时间分辨率的自动调整大小的范围，建议使用`dt_ref 的`10%       |
| min_samples                       | 3      | 最小样本数（应始终大于 2）                                   |
| global_plan_overwrite_orientation | true   | 覆盖全局规划器提供的局部子目标的方向（因为它们通常只提供 2D 路径） |
| global_plan_viapoint_sep          | -0.1   | 如果为正，则从全局规划中提取via-points（路径跟随模式）。该值决定了参考路径的分辨率（沿全局平面每两个连续via-points 之间的最小间隔，如果为负：禁用）。参考参数`weight_viapoint`调整强度。 **New in version 0.4** |
| max_global_plan_lookahead_dist    | 3.0    | 指定考虑优化的全局计划子集的最大长度（累积欧几里德距离）。实际长度由局部costmap大小和这个限值的逻辑结合决定。设置为零或负数以取消激活此限制。 |
| force_reinit_new_goal_dist        | 1.0    | 如果先前的目标更新间隔超过指定值（跳过热启动），则重新初始化轨迹 |
| feasibility_check_no_poses        | 4      | 指定每个采样间隔应检查预测计划上的哪个姿势的可行性           |
| publish_feedback                  | false  | 发布包含完整轨迹和活动障碍物列表的规划器反馈（应仅在评估或调试时启用） |
| shrink_horizon_backup             | true   | 允许规划器在自动检测到问题（例如不可行性）的情况下临时缩小范围 (50%)。另见参数`shrink_horizon_min_duration` |
| allow_init_with_backwards_motion  | false  | 如果为 true，则可能会使用向后运动初始化基础轨迹，以防目标在本地costmap中的起点后面（仅当机器人配备有后部传感器时才建议这样做） |
| exact_arc_length                  | false  | 如果为 true，则规划器在速度、加速度和转弯率计算中使用精确的弧长（-> 增加 cpu 时间），否则使用欧几里得距离近似 |
| shrink_horizon_min_duration       | 10.0   | 如果检测到不可行的轨迹，请指定缩小horizon的最短持续时间（请参阅参数`shrink_horizon_backup`以激活缩小地平线模式） |

#### 3.2.4、障碍物参数

| 参数名                                      | 默认值 | 参数含义                                                     |
| ------------------------------------------- | ------ | ------------------------------------------------------------ |
| min_obstacle_dist                           | 0.5    | 与障碍物的最小期望距离（m）                                  |
| include_costmap_obstacles                   | true   | 指定是否应考虑costmap的障碍。每个标记为障碍物的单元格都被视为一个点障碍物。因此不要选择非常小的costmap分辨率，因为它会增加计算时间。在未来的版本中，这种情况将得到解决，并为动态障碍提供额外的 api。 |
| costmap_obstacles_behind_robot_dist         | 1.0    | 限制在机器人后面进行规划时考虑到的占用的局部costmap障碍（m） |
| obstacle_poses_affected                     | 30     | 每个障碍物位置都附加到轨迹上最近的姿势以保持距离。也可以考虑额外的邻居。请注意，此参数可能会在未来版本中删除，因为在 kinetic+ 中已修改了障碍关联策略。参考`legacy_obstacle_association`的参数说明 |
| inflation_dist                              | 0.6    | 具有非零惩罚成本的障碍物周围的缓冲区（应大于`min_obstacle_dist`才能生效）。另请参阅权重`weight_inflation`。 |
| include_dynamic_obstacles                   | false  | 如果此参数设置为 true，则在优化过程中通过恒定速度模型预测和考虑具有非零速度的障碍物的运动 |
|                                             |        |                                                              |
| legacy_obstacle_association                 | false  | 连接轨迹姿势与优化障碍的策略已被修改（参见变更日志）。您可以通过将此参数设置为`true`来切换到旧/以前的策略。旧策略：对于每个障碍物，找到最近的TEB姿势；新策略：对于每个 teb 姿势，只找到“相关”的障碍 |
| obstacle_association_force_inclusion_factor | 1.5    | 非遗留障碍关联策略试图在优化过程中仅将相关障碍与离散化轨迹连接起来。但是指定距离内的所有障碍物都被强制包括在内（作为`min_obstacle_dist 的倍数`）。例如，选择 2.0 以在`2.0*` min_obstacle_dist`的半径内强制考虑障碍`。[仅当参数`legacy_obstacle_association`为`false`时才使用此参数] |
| obstacle_association_cutoff_factor          | 5      | obstacle_association_force_inclusion_factor`，但超出[value]* `min_obstacle_dist`的所有障碍无，在优化过程中被忽略。[仅当参数`legacy_obstacle_association`为`false`时才使用此参数] |

以下参数仅在需要[costmap_converter](http://wiki.ros.org/costmap_converter)插件时才相关（参见教程）：

| 参数名                        | 默认值 | 参数含义                                                     |
| ----------------------------- | ------ | ------------------------------------------------------------ |
| costmap_converter_plugin      | ""     | 定义插件名称以将costmap单元格转换为点/线/多边形。设置一个空字符串以禁用转换，以便将所有单元格视为点障碍 |
| costmap_converter_spin_thread | true   | 如果设置为 true，costmap 转换器会在不同的线程中调用它的回调队列 |
| costmap_converter_rate        | 5.0    | 定义 costmap_converter 插件处理当前cost的频率（该值不应高于cost更新率）[Hz] |

#### 3.2.5、优化参数

| 参数名                           | 默认值 | 参数含义                                                     |
| -------------------------------- | ------ | ------------------------------------------------------------ |
| no_inner_iterations              | 5      | 每次外循环迭代中调用的实际求解器迭代次数。请参阅参数`no_outer_iterations` |
| no_outer_iterations              | 4      | 每个外循环迭代都会根据所需的时间分辨率`dt_ref`自动调整轨迹大小并调用内部优化器（执行`no_inner_iterations`）。因此，每个规划周期中求解器迭代的总数是两个值的乘积 |
| penalty_epsilon                  | 0.1    | 为硬约束逼近的惩罚函数添加一个小的安全裕度                   |
| weight_max_vel_x                 | 2.0    | 满足最大允许平移速度的优化权重                               |
| weight_max_vel_theta             | 1.0    | 满足最大允许角速度的优化权重                                 |
| weight_acc_lim_x                 | 1.0    | 满足最大允许平移加速度的优化权重                             |
| weight_acc_lim_theta             | 1.0    | 满足最大允许角加速度的优化权重                               |
| weight_kinematics_nh             | 1000.0 | 用于满足非完整运动学的优化权重（此参数必须很高，因为运动学方程构成了等式约束，由于与其他成本相比，“原始”成本值较小，因此即使值为 1000 也不意味着矩阵条件不好） |
| weight_kinematics_forward_drive  | 1.0    | 强制机器人仅选择向前方向（正平移速度）的优化权重。小重量（例如 1.0）仍然允许向后行驶。1000 左右的值几乎可以防止向后行驶（但不能保证） |
| weight_kinematics_turning_radius | 1.0    | 强制最小转弯半径的优化权重（仅适用于汽车机器人）             |
| weight_optimaltime               | 1.0    | 缩短轨迹wrt转换/执行时间的优化权重                           |
| weight_obstacle                  | 50.0   | 与障碍物保持最小距离的优化权重                               |
| weight_viapoint                  | 1.0    | 用于最小化到via-point的距离的优化权重（相应的参考路径）。**0.4 新版本** |
| weight_inflation                 | 0.1    | 通货膨胀惩罚的优化权重（应该要很小）                         |
| weight_adapt_factor              | 2.0    | 在每次外部 TEB 迭代（weight_new = weight_old*factor）中，一些特殊的权重（当前是`weight_obstacle`）被这个因子重复缩放。迭代地增加权重而不是设置一个巨大的先验值会导致基础优化问题的更好的数值条件 |

#### 3.2.6、独特拓扑中的并行规划

| 参数名                          | 默认值 | 参数含义                                                     |
| ------------------------------- | ------ | ------------------------------------------------------------ |
| enable_homotopy_class_planning  | true   | 在不同的拓扑中激活并行规划（需要更多的 CPU 资源，因为一次优化多个轨迹） |
| enable_multithreading           | true   | 激活多线程以便在不同线程中规划每个轨迹                       |
| max_number_classes              | 4      | 指定考虑的不同轨迹的最大数量（限制计算工作量）               |
| selection_cost_hysteresis       | 1.0    | 指定新候选者必须具有与先前选择的轨迹相比多少轨迹成本才能被选中（如果 new_cost < old_cost*factor 则选择） |
| selection_obst_cost_scale       | 100.0  | 仅用于选择“最佳”候选者的障碍成本项的额外缩放。               |
| selection_viapoint_cost_scale   | 1.0    | 仅用于选择“最佳”候选者的额外缩放通过点成本条款。**0.4 新版本** |
| selection_alternative_time_cost | false  | 如果为真，则时间成本（时间差的平方和）将替换为总转换时间（时间差的总和） |
| roadmap_graph_no_samples        | 15     | 指定为创建roadmap graph生成的样本数量                        |
| roadmap_graph_area_width        | 6      | 在起点和目标之间的矩形区域中对随机关键点/航路点进行采样。以米为单位指定该区域的宽度 |
| h_signature_prescaler           | 1.0    | 用于区分同伦类的比例内部参数 ( *H-signature* )。警告：只减小这个参数，如果你观察到局部costmap中障碍物过多的问题，不要选择过低，否则无法区分障碍物（0.2<*值*<=1） |
| h_signature_threshold           | 0.1    | 如果实部和复部的差异都低于指定的阈值，则假定两个 H 签名相等  |
| obstacle_heading_threshold      | 1.0    | 指定障碍航向和目标航向之间的标量积的值，以便在探索时考虑它们（障碍物） |
| visualize_hc_graph              | false  | 可视化为探索独特轨迹而创建的图形（在 rviz 中检查标记消息）   |
| viapoints_all_candidates        | true   | 如果为真，则所有不同拓扑的轨迹都附加到一组通孔点，否则只有与初始/全局计划共享相同拓扑的轨迹与它们连接（对*test_optim_node*没有影响）。**0.4 新版本** |
| switching_blocking_period       | 0.0    | 指定在允许切换到新的等价类之前需要到期的持续时间（以秒为单位） |

#### 3.2.7、其他参数

| 参数名     | 默认值     | 参数含义                                                 |
| ---------- | ---------- | -------------------------------------------------------- |
| odom_topic | " `odom` " | 里程计消息的主题名称，由机器人驱动程序或模拟器提供       |
| map_frame  | " `odom` " | 全局规划框架（如果是静态地图，这个参数通常必须改为“/map” |

## 4、路线图

目前计划用于未来的一些功能和改进。 欢迎投稿！

- 在不可避免的障碍物（例如，位于非常靠近目标的障碍物）的情况下添加和改进安全功能。
- 实施适当的逃生行为。
- 针对规划器在多个局部最优解之间振荡的情况的改进/解决方案（不是基于拓扑，而是由于出现的噪声等）。

# teb_local_planner tutorial

参考链接：[teb_local_planner官方Tutorials](http://wiki.ros.org/teb_local_planner/Tutorials)

teb_local_planner/Tutorials目录

1. [设置和测试优化](http://wiki.ros.org/teb_local_planner/Tutorials/Setup and test Optimization)

   在本教程中，您将学习如何运行轨迹优化以及如何更改基础参数以设置自定义行为和性能。

2. [检查优化反馈](http://wiki.ros.org/teb_local_planner/Tutorials/Inspect optimization feedback)

   在本教程中，您将学习如何检查优化轨迹的反馈；给出了一个示例，该示例将当前所选轨迹的速度分布可视化。

3. [配置和运行机器人导航](http://wiki.ros.org/teb_local_planner/Tutorials/Configure and run Robot Navigation)

   在本教程中，您将学习如何将 teb_local_planner 设置为导航堆栈的本地规划器插件。

4. [避障和机器人足迹模型](http://wiki.ros.org/teb_local_planner/Tutorials/Obstacle Avoidance and Robot Footprint Model)

   在本教程中，您将学习如何实现避障。描述了主要关注机器人足迹模型及其影响的必要参数设置。

5. [遵循全局规划（Via-Points）](http://wiki.ros.org/teb_local_planner/Tutorials/Following the Global Plan (Via-Points))（Via-Points）

   在本教程中，您将学习如何配置本地规划器以更严格地遵循全局规划。特别是，您将学习如何在时间最优性和路径跟踪之间进行权衡。

6. [costmap转换](http://wiki.ros.org/teb_local_planner/Tutorials/Costmap conversion)

   在本教程中，您将学习如何应用 costmap 转换插件将占用的 costmap2d 单元转换为几何图元以进行优化（实验）。

7. [类汽车机器人的规划](http://wiki.ros.org/teb_local_planner/Tutorials/Planning for car-like robots)

   在本教程中，您将学习如何为类似汽车的机器人设置规划器（实验性）。

8. [完整机器人的规划](http://wiki.ros.org/teb_local_planner/Tutorials/Planning for holonomic robots)

   在本教程中，您将学习如何为完整机器人设置规划器（实验性）。

9. [加入定制的障碍物](http://wiki.ros.org/teb_local_planner/Tutorials/Incorporate customized Obstacles)

   在本教程中，您将学习如何考虑从其他节点发布的多边形障碍物。

10. [纳入动态障碍](http://wiki.ros.org/teb_local_planner/Tutorials/Incorporate dynamic obstacles)

    在本教程中，您将学习如何考虑从其他节点发布的动态障碍。

11. [通过 costmap_converter 跟踪并包含动态障碍物](http://wiki.ros.org/teb_local_planner/Tutorials/Track and include dynamic obstacles via costmap_converter)

    在本教程中，您将学习如何利用成本图转换器根据成本图更新轻松跟踪动态障碍物。

12. [经常问的问题](http://wiki.ros.org/teb_local_planner/Tutorials/Frequently Asked Questions)

    此页面尝试回答和解释有关 teb_local_planner 的常见问题。

## 1、安装

