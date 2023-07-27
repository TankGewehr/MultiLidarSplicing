# MultiLidarSplicing

用于根据多个激光雷达之间标定参数拼接多个激光雷达点云的工具

#
## 软件环境

* ros noetic
* eigen 3.3.7
* pcl 1.10

#
## 目录结构

```
MultiLidarSplicing
├─.vscode           vscode配置文件，可供debug时参考
├─app               应用程序源文件
├─config            应用程序配置文件以及传感器标定参数文件
├─include           头文件
├─lib               库文件
├─script            一些方便使用的脚本，可供使用时参考
├─src               源文件
├─.gitignore        gitignore配置文件
├─CMakeLists.txt    CmakeLists配置文件
└─README.md         使用及说明文档
```

#
## 编译

```shell
cd {MultiLidarSplicing}
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make
```

完成编译后生成的可执行文件位于`{MultiLidarSplicing}/bin`目录下，库文件位于`{MultiLidarSplicing}/lib`目录下

## 仅编译可执行文件

如果没有源代码并且有已经编译过的库文件，可以使用以下方法编译可执行文件

```shell
cd {MultiLidarSplicing}
mkdir build
cd build
cmake ..
make
```

#
## 参数说明

```
标定参数文件，可以参考{MultiLidarSplicing}/config/sensors/rslidar_front_middle.json
{
   "channel" : 传感器自身的rostopic,
   "modality" : 传感器自身类型,
   "image_size" : 无意义,
   "intrinsic" : 无意义,
   "distortion" : 无意义,
   "undistort_intrinsic" : 无意义,
   "undistort_distortion" : 无意义,
   "target" : 平移和旋转相对的目标传感器的rostopic,
   "rotation" : 旋转矩阵，表示该传感器在目标的坐标系下的旋转,
   "translation" : 平移向量，表示该传感器在目标的坐标系下的平移 [x,y,z]
}
```

```
应用程序配置文件，可以参考{MultiLidarSplicing}/config/config.json
{
    "calibration_params_path":{
        "lidar_front_left": 左前激光雷达标定参数文件路径,
        "lidar_front_middle": 前中激光雷达标定参数文件路径,
        "lidar_front_right": 右前激光雷达标定参数文件路径,
        "lidar_rear_left": 左后激光雷达标定参数文件路径,
        "lidar_rear_middle": 后中激光雷达标定参数文件路径,
        "lidar_rear_right": 右后激光雷达标定参数文件路径
    },
    "frame_id": 拼接点云frame_id,
    "publish_topic": 拼接点云rostopic,
}
```

#
## 使用教程

主要分为环境配置、运行两个部分

## 1、环境配置

为了能够接受到各个激光雷达的数据，需要配置其他设备的IP、hostname和ROS_MASTER_URI

``` shell
sudo vi /etc/hosts
```

在打开的文件末尾换行后添加以下内容，如果有多个设备则重复此步骤

```
{其他设备的IP} {其他设备的hostname}
```

配置ROS_MASTER_URI，对于每一个新开启的终端都需要重新配置

``` shell
export ROS_MASTER_URI=http://{启动roscore设备的IP}:11311
```

## 2、运行

```shell
cd {MultiLidarSplicing}
./bin/main ./config/config.json
```