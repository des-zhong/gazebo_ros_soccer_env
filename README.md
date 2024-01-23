配置：ubuntu20.04 5.15.0-91-generic, ros(neotic),gazebo 11

安装：

```
mkdir ws
cd ws
mkdir src
git clone ...
cd ..
catkin build
```

最后文件数结构为：

- ws
  - build
  - devel
  - logs
  - src
    - my_robot_controller

使用（每个终端都需要`source devel/setup.bash`后使用）：

启动环境：

```
roslaunch my_robot_controller load_env.launch
```

机器人运动(代码在my_robot_controller/src/robot_controller.py)：

```
roslaunch my_robot_controller robot_start.launch
```

