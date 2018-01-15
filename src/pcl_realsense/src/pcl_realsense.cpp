#include <iostream>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>


int i = 0;
//pcl::visualization::CloudViewer viewer("viewer");

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{

  ////save ten maps of pointclouds
  if(i<1)
  {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg (*input, cloud);
    char stt1[20] = "model_ratio/r";
    char stt2[2];
    sprintf(stt2, "%d", i);
    char stt3[5] = ".pcd";
    strcat(stt1,stt2);
    strcat(stt1,stt3);
    std::cout<<"stt1 = "<<stt1<<std::endl;
    pcl::io::savePCDFileASCII(stt1,cloud);
    std::cerr << "Saved " << cloud.points.size () << " data points to test_pcd.pcd." << std::endl;
    i++;
  }

//  //dispaly pointcloud realtime
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::fromROSMsg (*input, *cloud);
//    viewer.showCloud(cloud);

}

int main (int argc, char** argv)
{
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;
  ros::Rate loop_rate(200);


  // Fill in the cloud data
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/camera/depth/points", 1, cloud_cb);
  while(ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return (0);
}
