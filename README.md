# 功能介绍
在 [ORB_SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3/) 的基础上对RGBD数据的输入增加了生成Mesh的功能，并在在数据集 [TUM rgbd-dataset](https://vision.in.tum.de/data/datasets/rgbd-dataset/download) 上做了测试。 

# 代码改动
生成3D点云的代码参考了[高翔的 ORBSLAM2_with_pointcloud_map](https://github.com/gaoxiang12/ORBSLAM2_with_pointcloud_map)，将点云变成Mesh的代码参考了[PCL实现泊松表面重建](https://blog.csdn.net/u014801811/article/details/79748003)。

# 编译测试
在Ubuntu 20.04系统下编译,除了原本依赖的OpenCV、Eigen3、Pangolin以外，增加了对PCL的第三方库依赖，均采用 **sudo apt install libXX** 的形式安装，且只编译使用了 **Examples/RGB-D/rgbd_tum** 应用。
![Image view_running.jpg](https://github.com/ichangjian/ORB_SLAM3_Mesh/blob/master/view_running.jpg)
运行如下命令测试，会看到上图的运行界面，运行结束后会在当前目录下得到
*CameraTrajectory.txt、KeyFrameTrajectory.txt、result_p3d.ply、result_mesh.ply*。

``` shell
./Examples/RGB-D/rgbd_tum ./Vocabulary/ORBvoc.txt ./Examples/RGB-D/TUM1.yaml path/rgbd_dataset_freiburg1_xyz ./Examples/RGB-D/associations/fr1_xyz.txt 
```
其中*result_p3d.ply、result_mesh.ply*可以通过**MeshLab**软件查看，如下图。
![Image view_result.jpg](https://github.com/ichangjian/ORB_SLAM3_Mesh/blob/master/view_result.jpg )










