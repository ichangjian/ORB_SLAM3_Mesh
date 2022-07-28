# 功能介绍

1. 在 [ORB_SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3/) 的基础上对RGBD数据的输入增加了生成Mesh的功能，并在在数据集 [TUM rgbd-dataset](https://vision.in.tum.de/data/datasets/rgbd-dataset/download) 上做了测试。
2. 在左右双目+IMU上增加了**中值积分**解算IMU位姿的功能，可以以IMU的帧率往外更新位姿，并在数据集[TUM Visual-Inertial Dataset](https://vision.in.tum.de/data/datasets/visual-inertial-dataset)上作了测试。

# 代码改动

1. 生成3D点云的代码参考了[高翔的 ORBSLAM2_with_pointcloud_map](https://github.com/gaoxiang12/ORBSLAM2_with_pointcloud_map)，将点云变成Mesh的代码参考了[PCL实现泊松表面重建](https://blog.csdn.net/u014801811/article/details/79748003)。
2. 以IMU频率输出位姿的代码参考了[IMU数据积分获得当前位姿](https://blog.csdn.net/hjwang1/article/details/108322181)。

# 编译测试

在Ubuntu 20.04系统下编译,除了原本依赖的OpenCV、Eigen3、Pangolin以外，增加了对PCL的第三方库依赖，均采用 **sudo apt install libXX** 的形式安装，且只编译使用了 **Examples/RGB-D/rgbd_tum** 应用。
![Image view_running.jpg](https://github.com/ichangjian/ORB_SLAM3_Mesh/blob/master/view_running.jpg)
运行如下命令测试，会看到上图的运行界面，运行结束后会在当前目录下得到
*CameraTrajectory.txt、KeyFrameTrajectory.txt、result_p3d.ply、result_mesh.ply*。

``` bash
# 在TUM上测试MESH
./Examples/RGB-D/rgbd_tum ./Vocabulary/ORBvoc.txt ./Examples/RGB-D/TUM1.yaml path/rgbd_dataset_freiburg1_xyz ./Examples/RGB-D/associations/fr1_xyz.txt 

# 在D455上测试
./Examples/RGB-D/rgbd_realsense_D435i ./Vocabulary/ORBvoc.txt ./Examples/RGB-D/RealSense_D455i.yaml 

```

其中*result_p3d.ply、result_mesh.ply*可以通过**MeshLab**软件查看，如下图。
![Image view_result.jpg](https://github.com/ichangjian/ORB_SLAM3_Mesh/blob/master/view_result.jpg )

在TUM数据集room2上测试输出IMU频率POSE，绘制了SaveTrajectoryEuRoC输出的20HZ的POSE和以IMU输出的200HZ POSE的位置对比如下。![Image imu_frequency_pose.png](https://github.com/ichangjian/ORB_SLAM3_Mesh/blob/master/imu_frequency_pose.png)

``` bash
# 在TUM上测试输出IMU频率POSE
./Examples/Stereo-Inertial/stereo_inertial_tum_vi Vocabulary/ORBvoc.txt Examples/Stereo-Inertial/TUM-VI.yaml path/dataset-room2_512_16/mav0/cam0/data path/dataset-room2_512_16/mav0/cam1/data Examples/Stereo-Inertial/TUM_TimeStamps/dataset-room2_512.txt path/dataset-room2_512_16/mav0/imu0/data.csv dataset-room2_512_stereoi 

# 在D455上测试
./Examples/Stereo-Inertial/stereo_inertial_realsense_D435i ./Vocabulary/ORBvoc.txt ./Examples/Stereo-Inertial/RealSense_D455i.yaml

```

如果要打印IMU的POSE可以在 **Tracking::processIMU** 函数中将注释取消或添加回调函数。
