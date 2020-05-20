[English document](../README.md)

## 相机雷达标定文档

本方案提供了一个手动校准Livox雷达和相机之间外参的方法，已经在Mid-40，Horizon和Tele-15上进行了验证。其中包含了计算相机内参，获得标定数据，优化计算外参和雷达相机融合应用相关的代码。本方案中使用了标定板角点作为标定目标物，由于Livox雷达非重复性扫描的特点，点云的密度较大，比较易于找到雷达点云中角点的准确位置。相机雷达的标定和融合也可以得到不错的结果。

以下链接可以下载用来标定内参和外参的一组示例数据，数据都已保存在了程序默认路径下。

[示例数据](https://terra-1-g.djicdn.com/65c028cd298f4669a7f0e40e50ba1131/Download/update/data.zip)

### 步骤1: 环境配置

(以下标定基于Ubuntu 64-bit 16.04环境)

#### 1.1 安装环境和驱动
安装ROS环境，安装 [Livox SDK](https://github.com/Livox-SDK/Livox-SDK) 和 [livox_ros_driver](https://github.com/Livox-SDK/Livox-SDK-ROS) 如已安装可以跳过此步骤。

```
# 安装Livox_SDK
git clone https://github.com/Livox-SDK/Livox-SDK.git
cd Livox-SDK
sudo ./third_party/apr/apr_build.sh
cd build && cmake ..
make
sudo make install

# 安装livox_ros_driver
git clone https://github.com/Livox-SDK/livox_ros_driver.git ws_livox/src
cd ws_livox
catkin_make
```

#### 1.2 安装依赖

如已安装可以跳过此步骤。

- [PCL 安装](http://www.pointclouds.org/downloads/linux.html)

- [Eigen 安装](http://eigen.tuxfamily.org/index.php?title=Main_Page)

- [Ceres-solver 安装](http://ceres-solver.org/)

#### 1.3 下载源码，编译准备

```
git clone https://github.com/Livox-SDK/livox_camera_lidar_calibration.git
cd camera_lidar_calibration
catkin_make
source devel/setup.bash
```


#### 1.4 程序节点概括

此项目包括如下节点:

- cameraCalib - 标定相机内参

- pcdTransfer - 将雷达点云rosbag转换成PCD文件

- cornerPhoto - 获得照片角点

- getExt1 - 计算外参节点１，只优化外参

- getExt2 - 计算外参节点２，同时优化内参和外参

- projectCloud - 把雷达点云投影到照片上

- colorLidar - 雷达点云着色

以下程序节点中如果想修改launch文件，需要到src/calibration/launch文件夹中找对应的launch文件。

### 步骤2: 相机内参标定

#### 2.1 前期准备

准备一块有黑白棋盘格的标定板(如下所示，可以打印出来)

<div align=center><img src="chess_board.png" style="zoom: 150%;" /></div>

安装MATLAB来计算外参，或者使用cameraCalib节点

调整参数

保证选取相机是针孔成像模型，并先调整相机本身的基本参数(比如焦距或者照片尺寸等)。案例中使用了[Livox Horizon激光雷达](https://www.livoxtech.com/horizon) 和 [海康工业相机MV-CE060-10UC](https://www.hikrobotics.com/vision/visioninfo.htm?type=42&oid=2627) ，如下图所示。为了保证照片和雷达点云的视野是一致的，需要根据FOV对照片像素有一定的修改， 因为Horizon的FOV(视场角)是81.7(水平) x 25.1(垂直)，所以案例中的照片使用的是1520(宽度) x 568(高度)像素。

<div align=center><img src="lidar_camera.png" style="zoom: 50%;" /></div>

#### 2.2 MATLAB标定

这个方法需要安装MATLAB来计算结果，如果不想使用MATLAB可以参考2.3条目。

要准备20张以上的照片数据，各个角度和位置都要覆盖，拍摄的时候不要距离太近(3米左右)，如下图所示。

<div align=center><img src="intrinsic_calib.png"></div>

使用MATLAB中的cameraCalibrator工具，经过计算后可以得到如下结果，我们需要第1,2和11个数据

<div align=center><img src="matlab_result.png" style="zoom:150%;" /></div>

#### 2.3 cameraCalib标定

(如已使用2.2 MATLAB标定可以跳过此步骤)

按照2.2中的方法获得照片数据后，配置cameraCalib.launch中对应的路径和参数，默认是把照片数据放在data/camera/photos下，然后在data/camera/in.txt中写入所有需要使用的照片名称，如下图所示

<div align=center><img src="camera.png"  /></div>

输入指令开始标定

```
roslaunch camera_lidar_calibration cameraCalib.launch
```

标定结果中会保存在data/camera/result.txt中，包括重投影误差，内参矩阵和畸变纠正参数。

#### 2.4 内参结果

1. 一个3x3的内参矩阵(IntrinsicMatrix) [[注 1]](#注解)
2. ５个畸变纠正参数 k1, k2, p1, p2, k3

###  步骤3:  标定准备和数据采集

#### 3.1 标定场景准备

本方案使用标定板的四个角点来作为目标物 [[注 2]](#注解)。标定场景最好选择一个较为空旷的环境，这样方便识别标定板，并且保证雷达到标定板有３米以上。这个案例中使用了低反射率泡棉制作的标定板(1m x 1.5m)。需要选取至少10个左右不同的角度和距离来摆放标定板(参考相机内参的标定物摆放)，左右位置和不同的偏向角度最好都有采集数据 [[注 3]](#注解)。

<div align=center><img src="board.png"></div>

<div align=center>不同角度位置摆放标定板</div>

#### 3.2 连接雷达

检查标定板角点是否在点云中，输入点云可视化的命令查看点云

```
roslaunch livox_ros_driver livox_lidar_rviz.launch
```

需要录制rosbag时输入另一个命令

```
roslaunch livox_ros_driver livox_lidar_msg.launch
```

注意根据链接 https://github.com/Livox-SDK/livox_ros_driver 确认保存的rosbag数据格式是customMsg，后续程序中处理rosbag是对应的“livox custom msg format”格式。

#### 3.3 连接相机

本方案中使用海康相机的上位机MVS打开相机，检查获取照片的质量，并检查标定板角点是否在照片中。

#### 3.4 采集照片和点云数据

1. 拍摄照片

2. 运行指令录制点云

```
rosbag record /livox/lidar
```

3. 每个位置保存一张照片和10s左右的rosbag即可。

4. 数据采集完成后，将照片放在data/photo文件夹下; 雷达rosbag放在data/lidar文件夹下。

### 步骤4: 标定数据获取

#### 4.1 参数设置

首先需要把步骤２得到的内参和畸变纠正参数以下图的格式保存在data/parameters/intrinsic.txt文件下 [[注 4]](#注解)。distortion下面对应５个畸变纠正参数，按顺序是k1和k2 (RadialDistortion)，p1和p2 (TangentialDistortion)，最后一个是k3，一般默认是０

<div align=center><img src="intrinsic_format.png"  /></div>

#### 4.2 获得照片中的角点坐标

1. 配置cornerPhoto.launch文件中的照片路径，运行

```
roslaunch camera_lidar_calibration cornerPhoto.launch
```

2. 程序会在UI中打开对应的照片 [[注 5]](#注解)。在这个UI界面上只要把鼠标移到标定板的各个角上，窗口左下角就会显示对应的坐标数据。确定一个顺序，一般从左上角的角点开始，逆时针旋转按顺序记录下四个角点坐标。

<div align=center><img src="photo.png"></div>

3. 记录完毕后选中显示的图片按任意键，进入坐标输入流程。把记录下的四个坐标”x y”按顺序输入，x和y中间要有空格(比如: “635 487”)，输入完成后输入”0 0”即可结束输入流程(如下图例所示)。程序会算出四个更精确的float类型坐标显示出来，并保存在data/corner_photo.txt中。然后按任意键结束整个流程。

<div align=center><img src="photo_corner.png"></div>

4. 更改cornerPhoto.launch文件中的照片路径，重复上述流程，直至获得所有照片的角点坐标。

#### 4.3 获得雷达点云中的角点坐标

1. 检查pcdTransfer.launch文件中的rosbag路径，设置rosbag的数量，并将rosbag以0.bag, 1.bag...命名。

2. 运行指令将rosbag批量转化成PCD文件，PCD文件默认保存在data/pcdFiles文件夹中

```
roslaunch camera_lidar_calibration pcdTransfer.launch
```

3. 使用pcl_viewer打开PCD文件，按住shift+左键点击即可获得对应的点坐标。注意和照片采用相同的角点顺序[[注 6]](#注解)。

```
pcl_viewer -use_point_picking xx.pcd
```

4. 将xyz角点坐标按如下格式保存在data/corner_lidar.txt中，将所有PCD文件中雷达点云的角点坐标保存下来。

<div align=center><img src="corner_lidar.png"></div>

除了pcl_viewer之外也可以根据个人使用习惯使用别的点云可视化程序，能获得角点坐标即可。

### 步骤５: 外参计算

#### 5.1 参数设置

外参计算节点会读取data/corner_photo.txt和data/corner_lidar.txt中的标定数据来计算外参，数据需要保存成特定的格式才能被外参计算节点正确读取。参考以下格式，每行数据只有超过10个字母程序才会将其读取为计算的参数，比如下图用来编号的１２３４，lidar0和0.bmp这些标题不会被读取为计算参数。程序读到空行就会停止读取参数开始计算，所以保存时不要空行。

<div align=center><img src="lidar_photo.png" style="zoom:80%;" /></div>

在计算前检查一下雷达和相机两个标定数据中是否每行对应的是同一个角点，并检查数据量是否相同[[注 7]](#注解)。

#### 5.2 外参计算getExt1节点

计算前在getExt1.launch文件中配置好外参初值 [[注 8]](#注解)，输入指令开始计算外参

```
roslaunch camera_lidar_calibration getExt1.launch
```

终端中可以看到每次迭代运算的cost，外参结果以齐次矩阵的格式保存到data/parameters/extrinsic.txt下。

<div align=center><img src="opt_result.png"></div>

可以根据优化后的残差和重投影误差评估一下得到的外参 [[注 9]](#注解)。

重投影会把误差较大的数据打印在屏幕上，可以剔除异常标定数据后再重新进行优化计算。

#### 5.3 外参计算getExt２节点

getExt1节点只优化外参，而getExt2节点在计算的时候会将一开始计算的内参作为初值和外参一起优化。输入指令程序会得到一个新的内参和外参，并用新的参数来进行重投影验证。

```
roslaunch camera_lidar_calibration getExt2.launch
```

一般使用getExt1节点即可，如果在外参初值验证过，并且异常值已经剔除后，优化还是有较大的残差，那么可以使用getExt2试一试。使用的前提需要保证标定数据量较大，并且要充分验证结果。

如果经过验证getExt2计算的结果确实更好，那么把新的内参更新在data/parameters/intrinsic.txt中。

### 步骤６: 结果验证与相关应用

#### 6.1 概述

获得外参后我们可以用两个常见的应用看一下融合的效果。第一个是将点云投影到照片上，第二个是点云的着色[[注 10]](#注解)。

#### 6.2 投影点云到照片

在projectCloud.launch文件中配置点云和照片的路径后，运行指令，将rosbag中一定数量的点投影到照片上并且保存成新的照片。

```
roslaunch camera_lidar_calibration projectCloud.launch
```

<div align=center><img src="projection.png"></div>

<div align=center>投影照片效果</div>

#### 6.3 点云着色

在colorLidar.launch文件中配置点云和照片的路径，运行指令，可以在rviz中检查着色的效果。

```
roslaunch camera_lidar_calibration colorLidar.launch
```

<div align=center><img src="color.png"></div>

<div align=center>着色效果</div>

### 注解

1. 内参矩阵的格式如下图所示，一般在(0,0);(0,2);(1,1);(1,2)四个位置有对应的值。

<div align=center><img src="intrinsic.png"></div>

2. 标定的基本原理是通过同一目标物在雷达坐标系下的xyz坐标和相机坐标系下的xy坐标来计算并获得之间的转换关系。因为角点在点云和照片中都是比较明显的目标物，这样可以减少标定的误差。

3. 也可以使用多个标定板或者可以让雷达识别的棋盘格标定板。

4. 注意格式不要有变动，不然需要在程序中的common.h文件中的getIntrinsic和getDistortion程序中修改相关的代码。注意MATLAB中得到的内参矩阵是转置矩阵，输入到配置文件中时注意一下各个参数的位置。

5. 显示的照片是用畸变纠正参数修正过的。检查照片修正的是否正确，比如下图中左边的照片修正的有问题，可能是畸变参数写错了。右边的照片修正是正常的。

<div align=center><img src="photo_compare.png"></div>

6. 打开pcl_viewer后可以输入"h"来获得指引。

7. 注意少于10个字段不能被读取为计算数据，如果点云xyz或者照片xy坐标比较短需要补足10个字段。

8. 程序中的默认初值是根据Livox激光雷达自身坐标系，雷达和相机的相对位置设置的，要根据情况进行修改。如果初值差的很大可能会导致不好的优化结果。

9. 如果迭代结束cost还是比较大的量级(比如10的4次方), 那可能是程序中初值设置的有问题，得到的只是一个局部最优解，那么需要重新设置初值计算。

10. 点云投影到照片是通过内外参矩阵将雷达的点云投影到照片对应的位置，点云的颜色会根据距离从近到远显示从蓝到红；点云的着色是通过点云的xyz和得到的内外参矩阵算出对应的相机像素坐标，获取到这个像素的RGB信息后再赋给点云显示出来，这样雷达点云可以显示真实的颜色。
