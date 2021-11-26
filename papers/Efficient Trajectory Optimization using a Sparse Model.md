# Efficient Trajectory Optimization using a Sparse Model

# 基于稀疏模型的高效轨迹优化

## 摘要：

"timed elastic band"方法通过对全局规划器生成的初始轨迹进行后续修改来优化机器人轨迹。在轨迹优化中考虑的目标包括但不限于**总体路径长度**、**轨迹执行时间**、**和障碍物的距离**、**通过中间路径点**以及**遵循机器人的动力学、运动学**和**几何约束**。"时间橡皮筋"明确考虑运动的时空方面的动态约束，如有限的机器人速度和加速度。轨迹规划是实时进行的，这样"时间橡皮筋"就能应对动态障碍物和运动约束。将“时间橡皮筋”表述为标量化的**多目标优化问题**。大多数目标都是都是局部的，只与一小部分参数相关，因为它们只依赖于几个连续的机器人状态。这种局部结构导致了一个**稀疏的系统矩阵**，这允许利用快速和有效的优化技术，如开源框架“g2o”来解决“时间橡皮筋”问题。**“g2o"稀疏系统求解器**已成功地应用于VSLAM问题。本文描述了g2o框架在使用”定时橡皮筋“修正轨迹地背景下地应用和适应性。仿真和实际机器人实验结果表明，该方法具有良好地鲁棒性和计算效率。



## 1.介绍

轨迹规划是在满足机器人运动学和动态运动约束的前提下，找到最优的无碰撞轨迹。这篇文章主要研究假设全局规划器预先生成初始可行路径的轨迹修正问题[1]。特别是在服务机器人的场景中，对预先规划路径的动态修改比离线轨迹规划更可取。通过整合最新的传感器数据来处理动态环境的变化在线修正，从而对轨迹进行局部细化。在大多数现实的应用中，由于局部的、不完整的地图和动态障碍，环境模型会不断变化，此外，在实时应用中，大规模全局路径的（重新）计算通常是不可行的。这个观察结果导致了局部修改路径的方法，例如[2]，[3]提出的"时间橡皮筋"。

后来，该方法被推广到非完整运动学[4],[5],[6]和多自由度机器人系统[7]。[8]提出了一种方法，其初始路径是使用了优化技术的变型。轨迹的沿着路径的速度没有得到优化。随着优化的进行，时间参数用于控制路径的修正。规划器考虑非完整约束。

## 参考文献：

[1] S. M. LaV alle, ”Planning Algorithms”. Cambridge University Press,Cambridge, U.K., 2006.

[2] S. Quinlan, O. Khatib, ”Elastic Bands: Connecting Path Planning and Control”, in Proc. of the IEEE Int. Conf. on Robotics and Automation (ICRA), pp. 802-807, 1993.

[3] S. Quinlan, ”Real-time modification of collision-free paths”, PhD thesis, Stanford University, 1994.
[4] M. Khatib, ”Sensor-based motion control for mobile robots”, Labora- toire d’Automatique et d’Analyse des Systèmes LAAS-CNRS, 1996.
[5] M. Khatib, H. Jaouni, R. Chatila, J. P. Laumond, ”Dynamic Path Modification for Car-Like Nonholonomic Mobile Robots”, in Proc.
of the IEEE Int. Conf. on Robotics and Automation (ICRA), 1997.
[6] B. Graf, J. M. H. Wandosell, C. Schaeffer, ”Flexible Path Planning for Nonholonomic Mobile Robots”, Fraunhofer Institute Manufacturing Engineering and Automation (IPA), 2001.
[7] O. Brock, O. Khatib, ”Executing Motion Plans for Robots with Many Degrees of Freedom in Dynamic Environments”, in Proc. of the IEEE Int. Conf. on Robotics and Automation (ICRA), pp. 1-6, 1998.
[8] F. Lamiraux, D. Bonnafous, O. Lefebvre, ”Reactive path deformation for nonholonomic mobile robots”, in IEEE Transactions on Robotics, Vol. 20, No. 6, pp. 967-977, 2004.
[9] H. Kurniawati, T. Fraichard, ”From path to trajectory deformation”, IEEE/RSJ Intl. Conference on Intelligent Robots and Systems (IROS), pp. 159-164, 2007.
[10] V. Delsart, T. Fraichard, ”Reactive Trajectory Deformation to Navigate Dynamic Environments”, in Proc. of the Second European Robotics Symposium (EUROS), Vol. 44, pp. 233-241, 2008.
[11] B. Lau, C. Sprunk, W. Burgard, ”Kinodynamic Motion Planning for Mobile Robots Using Splines”, IEEE/RSJ Intl. Conference on
Intelligent Robots and Systems (IROS), pp. 2427-2433, 2009.
[12] C. Sprunk et al. ”Online Generation of Kinodynamic Trajectories for Non-Circular Omnidirectional Robots”, in Proc. of the IEEE Intl. Conference on Robotics and Automation (ICRA), pp. 72-77, 2011.
[13] C. Sprunk et al., ”Improved Non-linear Spline Fitting for Teaching Trajectories to Mobile Robots”, in Proc. of the IEEE Intl. Conference on Robotics and Automation (ICRA), pp. 2068-2073, 2012.
[14] J. Mattingley, Y. Wang, S. Boyd, ”Receding Horizon Control: Automatic Generation of High-Speed Solvers”, in IEEE Control Systems Magazine, Vol. 31, No. 3, pp. 52-65, 2011.
[15] N. Ratliff et al. ”CHOMP: Gradient Optimization Techniques for Efficient Motion Planning”, in IEEE Intl. Conference on Robotics and Automation (ICRA), May 2009.
[16] M. Kalakrishnan et al. ”STOMP: Stochastic trajectory optimization for motion planning”, in IEEE Intl. Conference on Robotics and Automation (ICRA), pp. 4569-4574, May 2011.
[17] C. Rösmann et al. ”Trajectory modification considering dynamic constraints of autonomous robots”, in Proceedings of the 7th German Conference on Robotics (ROBOTIK 2012). May 2012.
[18] K. Konolige, ”Sparse Bundle Adjustment”, in F. Labrosse et al.,editors, Proc. of the British Machine Vision Conference, pages 102.1-102.11. BMVA Press, September 2010.
[19] R. Kümmerle et al., ”g2o: A general framework for graph optimization”, in Proc. of the IEEE Intl. Conf. on Robotics and Automation (ICRA), Shanghai, China, May 2011.
[20] P. R. Amestoy, T. A. Davis, and I. S. Duff, ”Algorithm 837: Amd, an approximate minimum degree ordering algorithm.”, in ACM Trans. Math. Softw. vol. 30, pp. 381-388, September 2004.
[21] Y. Chen et al., ”Alogithm 887: Cholmod, supernodal sparse cholesky factorization and update/downdate”, in ACM Trans. Math. Softw. vol. 35, pp: 22:1-22:14, October 2008.
[22] L. E. Kavraki et al., ”Probabilistic roadmaps for path planning in high-dimensional configuration spaces”, in IEEE Transactions on Robotics and Automation, Vol. 12, No.4, pp.566-580, August 1996.