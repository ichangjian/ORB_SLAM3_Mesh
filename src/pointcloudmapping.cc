/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2016  <copyright holder> <email>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "pointcloudmapping.h"

#include <KeyFrame.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/poisson.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <boost/make_shared.hpp>
#include <boost/thread/thread.hpp>
#include <iostream>
#include <string>
#include <vector>

#include "Converter.h"

PointCloudMapping::PointCloudMapping(double resolution_) {
  this->resolution = resolution_;
  voxel.setLeafSize(resolution, resolution, resolution);
  globalMap = boost::make_shared<PointCloud>();

  viewerThread = make_shared<thread>(bind(&PointCloudMapping::viewer, this));
}

void PointCloudMapping::shutdown() {
  {
    unique_lock<mutex> lck(shutDownMutex);
    shutDownFlag = true;
    keyFrameUpdated.notify_one();
  }

  // pcl::io::savePCDFile("result_p3d.pcd", *globalMap);
  pcl::io::savePLYFile("result_p3d.ply", *globalMap);
  pcl::io::savePLYFile("result_mesh.ply", mesh_);
  cout << "globalMap save finished" << endl;
  viewerThread->join();
}

void PointCloudMapping::insertKeyFrame(KeyFrame* kf, cv::Mat& color,
                                       cv::Mat& depth) {
  cout << "receive a keyframe, id = " << kf->mnId << endl;
  unique_lock<mutex> lck(keyframeMutex);
  keyframes.push_back(kf);
  colorImgs.push_back(color.clone());
  depthImgs.push_back(depth.clone());

  keyFrameUpdated.notify_one();
}

pcl::PointCloud<PointCloudMapping::PointT>::Ptr
PointCloudMapping::generatePointCloud(KeyFrame* kf, cv::Mat& color,
                                      cv::Mat& depth) {
  PointCloud::Ptr tmp(new PointCloud());
  // point cloud is null ptr
  for (int m = 0; m < depth.rows; m += 3) {
    for (int n = 0; n < depth.cols; n += 3) {
      float d = depth.ptr<float>(m)[n];
      if (d < 0.01 || d > 10) continue;
      PointT p;
      p.z = d;
      p.x = (n - kf->cx) * p.z / kf->fx;
      p.y = (m - kf->cy) * p.z / kf->fy;

      p.b = color.ptr<uchar>(m)[n * 3];
      p.g = color.ptr<uchar>(m)[n * 3 + 1];
      p.r = color.ptr<uchar>(m)[n * 3 + 2];

      tmp->points.push_back(p);
    }
  }

  Eigen::Isometry3d T = ORB_SLAM3::Converter::toSE3Quat(kf->GetPose());
  PointCloud::Ptr cloud(new PointCloud);
  pcl::transformPointCloud(*tmp, *cloud, T.inverse().matrix());
  cloud->is_dense = false;

  cout << "generate point cloud for kf " << kf->mnId
       << ", size=" << cloud->points.size() << endl;
  return cloud;
}

void PointCloudMapping::viewer() {
  pcl::visualization::CloudViewer viewer3D("viewer");
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewerMesh(
      new pcl::visualization::PCLVisualizer("3D viewer"));
  viewerMesh->setBackgroundColor(0.5, 0.5, 1);

  viewerMesh->addCoordinateSystem(0.10);
  viewerMesh->initCameraParameters();
  viewerMesh->setCameraPosition(0, 0, 0, 0, 0, 10, 0, -1, 0);

  while (1) {
    {
      unique_lock<mutex> lck_shutdown(shutDownMutex);
      if (shutDownFlag) {
        break;
      }
    }
    {
      unique_lock<mutex> lck_keyframeUpdated(keyFrameUpdateMutex);
      keyFrameUpdated.wait(lck_keyframeUpdated);
    }

    // keyframe is updated
    size_t N = 0;
    {
      unique_lock<mutex> lck(keyframeMutex);
      N = keyframes.size();
    }

    clock_t tStart = clock();
    for (size_t i = lastKeyframeSize; i < N; i++) {
      PointCloud::Ptr p =
          generatePointCloud(keyframes[i], colorImgs[i], depthImgs[i]);
      *globalMap += *p;
    }
    std::cout << "=====================N " << N << "\n";
    lastKeyframeSize = N;
    PointCloud::Ptr tmp(new PointCloud());
    voxel.setInputCloud(globalMap);
    voxel.filter(*tmp);
    globalMap->swap(*tmp);

    if (globalMap->points.size() > 1e3) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
          new pcl::PointCloud<pcl::PointXYZ>);
      pcl::copyPointCloud(*globalMap, *cloud);

      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downSampled(
          new pcl::PointCloud<pcl::PointXYZ>);
      pcl::VoxelGrid<pcl::PointXYZ> downSampled;  //创建滤波对象
      downSampled.setInputCloud(cloud);  //设置需要过滤的点云给滤波对象
      downSampled.setLeafSize(0.01f, 0.01f,
                              0.01f);  //设置滤波时创建的体素体积为1cm的立方体
      downSampled.filter(*cloud_downSampled);  //执行滤波处理，存储输出

      // 统计滤波
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(
          new pcl::PointCloud<pcl::PointXYZ>);
      pcl::StatisticalOutlierRemoval<pcl::PointXYZ>
          statisOutlierRemoval;  //创建滤波器对象
      statisOutlierRemoval.setInputCloud(cloud_downSampled);  //设置待滤波的点云
      statisOutlierRemoval.setMeanK(50);  //设置在进行统计时考虑查询点临近点数
      statisOutlierRemoval.setStddevMulThresh(
          3.0);  //设置判断是否为离群点的阀值:均值+1.0*标准差
      statisOutlierRemoval.filter(
          *cloud_filtered);  //滤波结果存储到cloud_filtered

      // 计算法向量
      pcl::PointCloud<pcl::Normal>::Ptr normals(
          new pcl::PointCloud<pcl::Normal>);  //存储估计的法线的指针
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
          new pcl::search::KdTree<pcl::PointXYZ>);
      tree->setInputCloud(cloud_filtered);

      pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;  //法线估计对象
      n.setInputCloud(cloud_filtered);
      n.setSearchMethod(tree);
      n.setKSearch(20);
      n.compute(*normals);  //计算法线，结果存储在normals中

      //将点云和法线放到一起
      pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(
          new pcl::PointCloud<pcl::PointNormal>);  //法向量点云对象指针
      pcl::concatenateFields(*cloud_filtered, *normals, *cloud_with_normals);

      //创建搜索树
      pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(
          new pcl::search::KdTree<pcl::PointNormal>);
      tree2->setInputCloud(cloud_with_normals);

      //创建Poisson对象，并设置参数
      pcl::Poisson<pcl::PointNormal> pn;
      pn.setConfidence(
          false);  //是否使用法向量的大小作为置信信息。如果false，所有法向量均归一化。
      pn.setDegree(2);  //设置参数degree[1,5],值越大越精细，耗时越久。
      pn.setDepth(
          8);  //树的最大深度，求解2^d x 2^d x
               // 2^d立方体元。由于八叉树自适应采样密度，指定值仅为最大深度。
      pn.setIsoDivide(8);  //用于提取ISO等值面的算法的深度
      pn.setManifold(
          false);  //是否添加多边形的重心，当多边形三角化时。
                   //设置流行标志，如果设置为true，则对多边形进行细分三角话时添加重心，设置false则不添加
      pn.setOutputPolygons(
          false);  //是否输出多边形网格（而不是三角化移动立方体的结果）
      pn.setSamplesPerNode(
          9);  //设置落入一个八叉树结点中的样本点的最小数量。无噪声，[1.0-5.0],有噪声[15.-20.]平滑
      pn.setScale(1.25);  //设置用于重构的立方体直径和样本边界立方体直径的比率。
      pn.setSolverDivide(8);  //设置求解线性方程组的Gauss-Seidel迭代方法的深度

      //设置搜索方法和输入点云
      pn.setSearchMethod(tree2);
      pn.setInputCloud(cloud_with_normals);

      //执行重构
      pn.performReconstruction(mesh_);

      static int mesh_flag = 1;
      if (mesh_flag == 1) {
        mesh_flag++;
        viewerMesh->addPolygonMesh(mesh_,
                                   ("Blade" + std::to_string(mesh_flag)));
      } else {
        viewerMesh->removePolygonMesh("Blade" + std::to_string(mesh_flag));
        mesh_flag++;
        viewerMesh->addPolygonMesh(mesh_, "Blade" + std::to_string(mesh_flag));
      }
      std::cout << "===============mesh_flag " << mesh_flag << "\n";
    }

    viewer3D.showCloud(globalMap);
    cout << "show global map, size=" << globalMap->points.size() << endl;
    printf("===============SPEND %.2fs\n",
           (double)(clock() - tStart) / CLOCKS_PER_SEC);
  }
}
