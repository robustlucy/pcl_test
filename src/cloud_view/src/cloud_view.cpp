#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types_conversion.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>


//int main(int argc, char** argv)
//{

//  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
//  if (pcl::io::loadPCDFile("/home/zb/BoZhan/PCL/r1.pcd", *cloud1) == -1) //* 读入PCD格式的文件，如果文件不存在，返回-1
//  {
//      PCL_ERROR("Couldn't read file test_pcd.pcd \n"); //文件不存在时，返回错误，终止程序。
//      return (-1);
//  }

//  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
//  if (pcl::io::loadPCDFile("/home/zb/BoZhan/PCL/r5.pcd", *cloud2) == -1) //* 读入PCD格式的文件，如果文件不存在，返回-1
//  {
//      PCL_ERROR("Couldn't read file test_pcd.pcd \n"); //文件不存在时，返回错误，终止程序。
//      return (-1);
//  }
//  pcl::visualization::CloudViewer viewer1("viewer1");
//  pcl::visualization::CloudViewer viewer2("viewer2");

//  viewer1.showCloud(cloud1);
//  viewer2.showCloud(cloud2);
//  while (!viewer1.wasStopped())
//  {

//  }
//   return (0);
//}


int main(int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/zb/BoZhan/PCL/model/model0002.pcd", *cloud1) == -1) //* 读入PCD格式的文件，如果文件不存在，返回-1
  {
      PCL_ERROR("Couldn't read file test_pcd.pcd \n"); //文件不存在时，返回错误，终止程序。
      return (-1);
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile("/home/zb/BoZhan/PCL/forward/r2.pcd", *cloud2) == -1) //* 读入PCD格式的文件，如果文件不存在，返回-1
  {
      PCL_ERROR("Couldn't read file test_pcd.pcd \n"); //文件不存在时，返回错误，终止程序。
      return (-1);
  }

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer1 (new pcl::visualization::PCLVisualizer ("3D Viewer1"));
//  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2 (new pcl::visualization::PCLVisualizer ("3D Viewer2"));
  viewer1->setBackgroundColor (0, 0, 0);
  viewer1->addPointCloud<pcl::PointXYZ> (cloud1, "model cloud");
//  viewer1->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud1");
  viewer1->addCoordinateSystem (0.01);
  viewer1->initCameraParameters ();

//  viewer2->setBackgroundColor (0, 0, 0);
//  viewer2->addPointCloud<pcl::PointXYZ> (cloud2, "sample cloud");
////  viewer2->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud2");
//  viewer2->addCoordinateSystem (0.01);
//  viewer2->initCameraParameters ();


  while (!viewer1->wasStopped ())
  {
    viewer1->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
//  while (!viewer1->wasStopped())
//  {

//  }
   return (0);
}

