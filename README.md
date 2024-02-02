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
或者
```
rosrun my_robot_controller robot_controller.py
```


训练时：直接使用脚本train_mappo.py（不需要手动launch文件）
首先杀死正在运行的ros进程(每次运行此脚本都需要进行一次，可能会有报错，也可以直接使用此命令终止训练)：
```
killall -9 rosout roslaunch rosmaster gzserver nodelet robot_state_publisher gzclient python python3
```
然后
```
python train_mappo.py
```
关闭可视化可增加训练速度：load_env.launch 中的gui选项变成false即可：
```
<arg name="gui" default="false"/>
```

