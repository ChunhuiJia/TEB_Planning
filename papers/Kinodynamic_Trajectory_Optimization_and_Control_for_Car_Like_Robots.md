# Kinodynamic Trajectory Optimization and Control for Car Like Robots

Christoph Rösmann, Frank Hoffmann and Torsten Bertram

## 摘要

本文提出了一种新颖的用于类车辆机器人在线运动规划的“时间弹性带”的通用公式。将规划问题定义为受机器人动力和运动学约束和障碍物回避的有限维度和稀疏优化问题。控制运动隐含包括在优化的轨迹里。动态环境下的可靠的导航是通过使用状态反馈增强内部优化循环来实现的。该预测控制方案具有实时性和对机器人感知区域内障碍物的相应能力。在大型和复杂环境中的导航是通过请求全局规划器的中间目标点以纯追踪的方式实现的。对初始全局路径的要求比较温和，不要求符合机器人运动学。通过与Reeds Shepp曲线的对比分析和对原型车辆操纵的研究说明了该方法的优点。

## 1.介绍

在移动机器人的背景下，轨迹规划和控制是服务机器人和自动交通系统等应用的基本任务。在线规划优于离线解决方案，因为前者把规划和状态反馈结合起来，并在运行时对动态环境和扰动作出反应。弹性带(Elastic band)是著名的在线路径变形方法[1]。预定义的内力使路径收缩，而外力使与障碍物分离。然而，传统的路径规划并没有明确地包含时间和运动和动力学约束。对于轨迹而不是路径的在线变形的EB方法的扩展在[2]中提出。离散的轨迹路点被障碍物排斥。在此基础上建立了动态运动模型。Delsart等人将这两个阶段合并为单个操作[3]。然而，基于在线轨迹优化的方法往往受到计算量的限制，无法在实时性的约束下收敛到可行的最优解。基于采用的方法，如动态窗口法(DWA)[4],解决了计算效率的问题。对模拟轨迹进行采样，由一组可行速度限制的速度搜索空间中反复搜索并评分，评分关于到目标的剩余距离，速度和障碍物的距离等。[5]根据机器人的动力学约束，优化由样条曲线表示的轨迹。

大多数规划器只考虑非完整约束的差分驱动机器人，而不考虑类机器人的最小转弯半径。一个关于反馈控制技术的概述在[6]中提供。Lamiraux等人提出了一种基于势梯度的路径变形[7]。该方法依赖于平滑的路径表示并且没有明确涵盖速度限制。这实际上限制了其直接用于停车操作。Vendittelli等人基于著名的Reeds Shepp曲线提供了点形机器人的无碰撞路径。RS曲线给出了运动学模型的最小时间最优控制问题的解析解[9]。此外，经典的EB算法已被用于类汽车机器人[10]。因此，将内外力作用的连续路径点通过RS曲线连接起来。此外，利用Bezier多项式进行更平滑的过渡。然而，变形不受任意速度和加速度的限制，因此需要时间缩放来确定可行的（尽管不是最优的）轨迹。Gu等人提出了一种多阶段规划方法，该方法利用无优化EB生成路径，然后进行速度规划阶段。在直接轨迹优化的背景下，DWA扩展到支持[12]中的类机器人，因为它限制了旋转速度的搜索空间到可行解集。然而，预测中恒定速度的假设禁止在有限空间中导航所需的反向运动。[13]中展示了一个非实时的，但完整的最优控制公式，考虑了运动学和动力学约束和旅行时间。其他基于搜索的策略，例如[14]中使用栅格分解或者对RRT搜索的路径应用平滑过滤器[15]。

“时间弹性带”（TEB）方法为差分驱动机器人提供了一种实时在线轨迹规划器[16],[17]。

## 参考文献：


[1] S. Quinlan,Real-Time Modification of Collision-Free Paths. Stanford,CA, USA: Stanford University, 1995.
[2] H. Kurniawati and T. Fraichard, “From path to trajectory deformation,” in IEEE-RSJ Int. Conf. on Intelligent Robots and Systems, 2007.
[3] V . Delsart and T. Fraichard, “Reactive Trajectory Deformation to Navigate Dynamic Environments,” inEuropean Robotics Symposium, Czech Republic, 2008.
[4] D. Fox, W. Burgard, and S. Thrun, “The dynamic window approach to collision avoidance,”IEEE Robotics & Automation Magazine, vol. 4, no. 1, pp. 23–33, March 1997.
[5] B. Lau, C. Sprunk, and W. Burgard, “Kinodynamic motion planning for mobile robots using splines,” inIEEE/RSJ Intl. Conf. on Intelligent Robots and Systems, USA, 2009, pp. 2427–2433.
[6] A. D. Luca, G. Oriolo, and C. Samson,Feedback control of a nonholonomic car-like robot. Springer, 1998, pp. 171–253.
[7] F. Lamiraux, D. Bonnafous, and O. Lefebvre, “Reactive path deformation for nonholonomic mobile robots,”IEEE Transactions on Robotics, vol. 20, no. 6, pp. 967–977, 2004.
[8] M. V endittelli, J. P . Laumond, and C. Nissoux, “Obstacle distance for car-like robots,”IEEE Transactions on Robotics and Automation, vol. 15, no. 4, pp. 678–691, 1999.
[9] S. LaV alle,Planning Algorithms. Cambridge University Press, 2006.
[10] M. Khatib, H. Jaouni, R. Chatila, and J. P . Laumond, “Dynamic path modification for car-like nonholonomic mobile robots,” inIntl. Conf on Robotics and Automation, vol. 4, 1997, pp. 2920–2925.
[11] T. Gu, J. Atwood, C. Dong, J. M. Dolan, and J.-W. Lee, “Tunable and stable real-time trajectory planning for urban autonomous driving,” in IEEE International Conference on Intelligent Robots and Systems, 2015, pp. 250–256.
[12] K. Rebai, O. Azouaoui, M. Benmami, and A. Larabi, “Car-like robot navigation at high speed,” inIEEE Intl. Conf. on Robotics and
Biomimetics, 2007, pp. 2053–2057.
[13] S. Gulati, “A framework for characterization and planning of safe, comfortable, and customizable motion of assistive mobile robots.” Ph.D. dissertation, 2011.
[14] N. Ghita and M. Kloetzer, “Trajectory planning for a car-like robot by environment abstraction,”Robotics and Autonomous Systems, vol. 60, no. 4, pp. 609–619, 2012.
[15] W. Xu and J. M. Dolan, “A real-time motion planner with trajectory optimization for autonomous vehicles,” inProceedings of the Intl. Conf. on Robotics and Automation, 2012, pp. 2061–2067.
[16] C. Rösmann, W. Feiten, T. Wösch, F. Hoffmann, and T. Bertram, “Trajectory modification considering dynamic constraints of autonomous robots,” in7th German Conference on Robotics, 2012, pp. 74–79.
[17] C. Rösmann, F. Hoffmann, and T. Bertram, “Integrated online trajectory planning and optimization in distinctive topologies,”Robotics and Autonomous Systems, vol. 88, pp. 142 – 153, 2017.
[18] C. Rösmann, “teblocalplanner ROS Package [Online],” 2015. [Online]. Available: http://wiki.ros.org/teblocalplanner
[19] J. Nocedal and S. J. Wright,Numerical optimization, ser. Springer series in operations research. New Y ork: Springer, 1999.
[20] R. Kümmerle, G. Grisetti, H. Strasdat, K. Konolige, and W. Burgard, “G2o: A general framework for graph optimization,” inIEEE Intl. Conf. on Robotics and Automation (ICRA), 2011, pp. 3607–3613.

