# Octomap_RRT_Dynamic
* 简易的基于octomap八叉树3D概率分布地图
* fcl碰撞检测
* Ompl机器人运动轨迹规划

## demo
```bash
-- octomap_processor # octomap_server功能包，可以支持直接将雷达点云/相机点云转换为 八叉树概率分布3D地图
-- ompl_rrt_planner  # ompl 运动控制功能包，结合fcl的碰撞检测（将octomap当中的tree转换为fcl的octree碰撞对象）生成带避障的机器人运行轨迹
```