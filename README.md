配置：ubuntu20.04 5.15.0-91-generic, ros(neotic),gazebo 11

创建workspace：

```
mkdir ws
cd ws
mkdir src
```
下载项目
```
git clone https://github.com/des-zhong/gazebo_ros_soccer_env.git
```
将gazebo_ros_soccer_env文件下下所有文件移动至src文件夹中，进入ws文件夹路径进行编译：
```
catkin build -DPYTHON_EXECUTABLE=/usr/bin/python3
```

最后文件数结构为：
- ws
  - build
  - devel
  - logs
  - src
    - my_robot_controller
  - train.py

由于world文件不支持相对路径因此需要复制环境文件deepsoccer_gazebo到.gazebo/models文件夹，进入ws/src/my_robot_controller路径：
```
cp -r ./deepsoccer_gazebo ~/.gazebo/models
```

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

文件说明：Command2Velocity脚本用于将离散的指令转化为gazebo_plugin脚本可以使用的速度

场地设置：长6（x轴），宽4（y轴），球门设置在宽侧，宽度为1.126

