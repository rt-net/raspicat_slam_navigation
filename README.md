# raspicat_slam

[![](https://i.gyazo.com/19308c7f22694d8a02b9a3dc3da4b5f5.jpg)](https://www.rt-shop.jp/blog/archives/11023)

## Usage

### Raspberry Pi on Raspberry Pi Cat

`slam_remote_robot_*_urg.launch` starts up nodes in `urg_node`, `raspicat_gamepad_controller`, `rt_usb_9axisimu_driver` and `raspicat` to control the robot and publish LiDAR scan data, IMU data.

* when USB URG is connected e.g.) [URG-04LX-UG01](https://www.hokuyo-aut.co.jp/search/single.php?serial=17), [UTM-30LX](https://www.hokuyo-aut.co.jp/search/single.php?serial=21)
```sh
roslaunch raspicat_slam slam_remote_robot_usb_urg.launch lidar_port:="/dev/ttyACM0" imu_port:="/dev/ttyACM1"
```
If LiDAR is recognized by a device other than `/dev/ttyACM0` and IMU is recognized by a device other than `/dev/ttyACM1` (for example, LiDAR is `/dev/ttyACM1` and IMU is `/dev/ttyACM0`). You can specify the device to boot as follows.
```sh
roslaunch raspicat_slam slam_remote_robot_usb_urg.launch lidar_port:="/dev/ttyACM1" imu_port:="/dev/ttyACM0"
```

* when Ether URG is connected e.g.) [UST-10LX](https://www.hokuyo-aut.co.jp/search/single.php?serial=16), [UST-20LX](https://www.hokuyo-aut.co.jp/search/single.php?serial=16)
```sh
roslaunch raspicat_slam slam_remote_robot_ether_urg.launch ip_address:="192.168.0.10"
```
* When not using the IMU.
 
For USB URG 
```
roslaunch raspicat_slam slam_remote_robot_usb_urg.launch lidar_port:="/dev/ttyACM0" imu_port:="/dev/ttyACM1" imu:="false"
```
For Ether URG 
```
roslaunch raspicat_slam slam_remote_robot_ether_urg.launch ip_address:="192.168.0.10" imu:="false"
```


### PC

`slam_remote_pc.launch` starts up nodes for gmapping to SLAM.

```sh
roslaunch raspicat_slam slam_remote_pc.launch
```

## License

This repository is released under the Apache License 2.0, see [LICENSE](./LICENSE).

Some of the launch files are delivered from "with_gamepad" branch on [ryuichiueda/raspimouse_slam](https://github.com/ryuichiueda/raspimouse_slam), released under the MIT License.
