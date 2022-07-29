# raspicat_slam_navigation
Raspberry Pi CatでSLAMやナビゲーションを行うためのROSメタパッケージです。

## Requirements
- Raspberry Pi Cat
  - Raspberry Pi
    - Raspberry Pi 4 Model B
  - Linux OS
    - [Ubuntu Server 18.04](https://ubuntu.com/download/raspberry-pi)
  - Device Driver
    - [rt-net/RaspberryPiMouse](https://github.com/rt-net/RaspberryPiMouse)
  - ROS
    - [Melodic Morenia](https://wiki.ros.org/melodic)

- Remote PC
  - Linux OS
    - [Ubuntu Desktop 18.04](https://ubuntu.com/download/desktop)
  - ROS
    - [Melodic Morenia](https://wiki.ros.org/melodic)

## Installation
### Source Build
#### Raspberry Pi 4

```sh
# パッケージのダウンロード
cd ~/catkin_ws/src
git clone -b $ROS_DISTRO-devel https://github.com/rt-net/raspicat_slam_navigation.git
git clone -b $ROS_DISTRO-devel https://github.com/rt-net/raspicat_description.git
git clone -b $ROS_DISTRO-devel https://github.com/rt-net/raspicat_ros.git
```

```sh
# 依存パッケージのインストール
rosdep update
rosdep install -r -y -i --from-paths .
```

```sh
# ビルド＆インストール
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash
```
#### Remote PC

```sh
# パッケージのダウンロード
cd ~/catkin_ws/src
git clone -b $ROS_DISTRO-devel https://github.com/rt-net/raspicat_slam_navigation.git
git clone -b $ROS_DISTRO-devel https://github.com/rt-net/raspicat_description.git
git clone -b $ROS_DISTRO-devel https://github.com/rt-net/raspicat_ros.git
git clone -b $ROS_DISTRO-devel https://github.com/rt-net/raspicat_sim.git  # シミュレータを使う場合
```

```sh
# 依存パッケージのインストール
rosdep update
rosdep install -r -y -i --from-paths .
```

```sh
# ビルド＆インストール
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash
```

## Package Overview
### raspicat_slam

SLAMを行うためのパッケージです。

このパッケージでは、以下のROSパッケージを使用してSLAMを実行することができます。
* [gmapping](http://wiki.ros.org/gmapping)
* [cartographer](http://wiki.ros.org/cartographer)
* [slam_toolbox](http://wiki.ros.org/slam_toolbox)

### raspicat_navigation

ナビゲーションを行うためのパッケージです。

このパッケージでは、以下のROSパッケージを使用してナビゲーションを実行することができます。
* [move_base](http://wiki.ros.org/move_base)
* [neonavigation](https://github.com/at-wat/neonavigation)

## Usage

### Rosbag play
#### SLAM

* Gmapping

```
roslaunch raspicat_slam raspicat_gmapping.launch rosbag:=true rosbag_rate:=1 rosbag_topics:="/odom /scan /tf /tf_static" rosbag_filename:=$(rospack find raspicat_slam)/config/rosbag/iscas_museum.bag
roslaunch raspicat_slam map_save.launch map_file:=$(rospack find raspicat_slam)/config/maps/gmapping
```

https://user-images.githubusercontent.com/40545422/166213251-22115ecb-11a2-4caa-95c7-c3b8e85db3ad.mp4

* Cartographer

```
roslaunch raspicat_slam raspicat_cartographer.launch rosbag:=true rosbag_rate:=1 rosbag_topics:="/odom /scan /tf /tf_static" rosbag_rate:=1 rosbag_filename:=$(rospack find raspicat_slam)/config/rosbag/iscas_museum.bag
roslaunch raspicat_slam map_save.launch map_file:=$(rospack find raspicat_slam)/config/maps/cartographer
```

https://user-images.githubusercontent.com/40545422/166214138-624d3fd0-2bf8-46d7-a722-1602007df086.mp4

* Slam Toolbox

```
roslaunch raspicat_slam raspicat_slam_toolbox.launch rosbag:=true rosbag_rate:=1 rosbag_topics:="/odom /scan /tf /tf_static" rosbag_rate:=1 rosbag_filename:=$(rospack find raspicat_slam)/config/rosbag/iscas_museum.bag
roslaunch raspicat_slam map_save.launch map_file:=$(rospack find raspicat_slam)/config/maps/slam_toolbox
```

https://user-images.githubusercontent.com/40545422/166214198-38cf95ee-57ce-45bf-82d4-447f93419560.mp4

### Simulation
#### SLAM

* Gmapping

```
roslaunch raspicat_gazebo raspicat_iscas_museum.launch rviz:=false
roslaunch raspicat_slam raspicat_gmapping.launch joy:=false
roslaunch raspicat_slam map_save.launch map_file:=$(rospack find raspicat_slam)/config/maps/gmapping
```

* Cartographer

```
roslaunch raspicat_gazebo raspicat_iscas_museum.launch rviz:=false
roslaunch raspicat_slam raspicat_cartographer.launch joy:=false
roslaunch raspicat_slam map_save.launch map_file:=$(rospack find raspicat_slam)/config/maps/cartographer
```

* Slam Toolbox

```
roslaunch raspicat_gazebo raspicat_iscas_museum.launch rviz:=false
roslaunch raspicat_slam raspicat_slam_toolbox.launch joy:=false
roslaunch raspicat_slam map_save.launch map_file:=$(rospack find raspicat_slam)/config/maps/slam_toolbox
```

#### Navigation

* move_base

```
roslaunch raspicat_gazebo raspicat_iscas_museum.launch rviz:=false
roslaunch raspicat_navigation raspicat_navigation.launch navigation:="move_base"
```

https://user-images.githubusercontent.com/40545422/166214286-c78d74d1-cab1-489d-85fc-68416d48c655.mp4

* Neonavigation
```
roslaunch raspicat_gazebo raspicat_iscas_museum.launch rviz:=false
roslaunch raspicat_navigation raspicat_navigation.launch navigation:="neonav"
```

https://user-images.githubusercontent.com/40545422/166214304-23606730-3d8e-4ed4-9f9d-7834a3788aba.mp4

### Actual machine
#### SLAM
* Gmapping
##### Raspberry Pi 4
```
roscore
roslaunch raspicat_bringup raspicat_bringup.launch lidar_ether:=false lidar_usb:=true joy:=true
```
##### Remote PC
```
roslaunch raspicat_slam raspicat_gmapping.launch
roslaunch raspicat_slam map_save.launch map_file:=$(rospack find raspicat_slam)/config/maps/gmapping
```

* Cartographer
##### Raspberry Pi 4
```
roscore
roslaunch raspicat_bringup raspicat_bringup.launch lidar_ether:=false lidar_usb:=true joy:=true
```
##### Remote PC
```
roslaunch raspicat_slam raspicat_cartographer.launch
roslaunch raspicat_slam map_save.launch map_file:=$(rospack find raspicat_slam)/config/maps/cartographer
```

* Slam Toolbox
##### Raspberry Pi 4
```
roscore
roslaunch raspicat_bringup raspicat_bringup.launch lidar_ether:=false lidar_usb:=true joy:=true
```
##### Remote PC
```
roslaunch raspicat_slam raspicat_slam_toolbox.launch
roslaunch raspicat_slam map_save.launch map_file:=$(rospack find raspicat_slam)/config/maps/slam_toolbox
```

#### Navigation
* Move_base
##### Raspberry Pi 4
```
roscore
roslaunch raspicat_bringup raspicat_bringup.launch lidar_ether:=false lidar_usb:=true joy:=false
```
##### Remote PC
```
roslaunch raspicat_navigation raspicat_navigation.launch navigation:="move_base"
```

* Neonavigation
##### Raspberry Pi 4
```
roscore
roslaunch raspicat_bringup raspicat_bringup.launch lidar_ether:=false lidar_usb:=true joy:=false
```
##### Remote PC
```
roslaunch raspicat_navigation raspicat_navigation.launch navigation:="neonav"
```

## License
(C) 2020-2022 RT Corporation

各ファイルはライセンスがファイル中に明記されている場合、そのライセンスに従います。特に明記されていない場合は、Apache License, Version 2.0に基づき公開されています。  
ライセンスの全文は[LICENSE](./LICENSE)または[https://www.apache.org/licenses/LICENSE-2.0](https://www.apache.org/licenses/LICENSE-2.0)から確認できます。

※このソフトウェアは基本的にオープンソースソフトウェアとして「AS IS」（現状有姿のまま）で提供しています。本ソフトウェアに関する無償サポートはありません。  
バグの修正や誤字脱字の修正に関するリクエストは常に受け付けていますが、それ以外の機能追加等のリクエストについては社内のガイドラインを優先します。
