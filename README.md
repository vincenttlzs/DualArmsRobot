# DualArmsRobot

1.把其中一个程序包文件夹放入catkin工作空间中编译运行便可。再把该程序包删除放入另外一个再次编译便可运行另外一个。
  本设计中运行的catkin工作空间路径为  /home/zs/catkin_ws

rviz+moveit仿真运行代码为

$roslaunch shuangx_moveit_config demo.launch
$rosrun shuangx_moveit_config model.py



2.gazebo物理环境与moveit交互的运行代码为

$roslaunch shuang shuang_robot.launch
$roslaunch shuang_moveit_config shuang_robot_moveit_planning_execution.launch sim:=true
$roslaunch shuang_moveit_config moveit_rviz.launch config:=true


