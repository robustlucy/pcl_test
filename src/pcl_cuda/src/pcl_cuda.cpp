//#include <pcl/gpu/octree/octree.hpp>
//#include <pcl/gpu/containers/device_array.hpp>
//#include <pcl/io/openni_grabber.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>
//#include <boost/shared_ptr.hpp>
//#include <pcl/visualization/cloud_viewer.h>
//#include <iostream>

//int main(int argc, char** argv)
//{
//  pcl::PointCloud<pcl::PointXYZ> cloud;
//  cloud.width    = 500;
//  cloud.height   = 200;
//  cloud.is_dense = false;

//  for (size_t w = 0; w < cloud.width; ++w)
//  {
//    for (size_t h = 0; h < cloud.height; ++h)
//    {
//      pcl::PointXYZ p;
//      p.x = w; p.y = h; p.z = 1;
//      cloud.points.push_back(p);
//    }
//  }

//  pcl::io::savePCDFileASCII ("input.pcd", cloud);
//  std::cout << "INFO: Saved " << cloud.points.size () << " data points to test_pcd.pcd." << std::endl;

//  pcl::gpu::Octree::PointCloud cloud_device;
//  cloud_device.upload(cloud.points);

//  pcl::gpu::Octree octree_device;
//  octree_device.setCloud(cloud_device);
//  octree_device.build();

//  // Create two query points
//  std::vector<pcl::PointXYZ> query_host;
//  query_host.resize (3);
//  query_host[0].x = 250;
//  query_host[0].y = 100;
//  query_host[0].z = 1;
//  query_host[1].x = 0;
//  query_host[1].y = 0;
//  query_host[1].z = 1;
//  query_host[2].x = 500;
//  query_host[2].y = 200;

//  pcl::gpu::Octree::Queries queries_device;
//  queries_device.upload(query_host);

//  // Take two identical radiuses
//  std::vector<float> radius;
//  radius.push_back(10.0);
//  radius.push_back(10.0);
//  radius.push_back(10.0);

//  pcl::gpu::Octree::Radiuses radiuses_device;
//  radiuses_device.upload(radius);

//  const int max_answers = 500*200;

//  // Output buffer on the device
//  pcl::gpu::NeighborIndices result_device(queries_device.size(), max_answers);

//  // Do the actual search
//  octree_device.radiusSearch(queries_device, radiuses_device, max_answers, result_device);

//  std::vector<int> sizes, data;
//  result_device.sizes.download(sizes);
//  result_device.data.download(data);

//  std::cout << "INFO: Data generated" << std::endl;

//  std::cout<< "INFO: found : " << data.size() << " data.size" << std::endl;
//  std::cout<< "INFO: found : " << sizes.size() << " sizes.size" << std::endl;

//  for (size_t i = 0; i < sizes.size (); ++i)
//  {
//    std::cout << "INFO: sizes : " << i << " size " << sizes[i] << std::endl;
//    if(sizes[i] != 0)
//    {
//      pcl::PointCloud<pcl::PointXYZ> cloud_result;
//      // Fill in the cloud data
//      cloud_result.height   = 1;
//      cloud_result.is_dense = false;

//      for (size_t j = 0; j < sizes[i] ; ++j)
//      {
//        cloud_result.points.push_back(cloud.points[data[j + i * max_answers]]);
//        std::cout << "INFO: data : " << j << " " << j + i * max_answers << " data " << data[j+ i * max_answers] << std::endl;
//      }
//      std::stringstream ss;
//      ss << "cloud_cluster_" << i << ".pcd";
//      cloud_result.width    = cloud_result.points.size();
//      pcl::io::savePCDFileASCII (ss.str(), cloud_result);
//      std::cout << "INFO: Saved " << cloud_result.points.size () << " data points to " << ss.str() << std::endl;
//    }
//  }
//  return 0;
//}


///*******************************example 2***************************************************//
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
// The GPU specific stuff here
#include <pcl/gpu/octree/octree.hpp>
#include <pcl/gpu/containers/device_array.hpp>
#include <pcl/gpu/segmentation/gpu_extract_clusters.h>
#include <pcl/gpu/segmentation/impl/gpu_extract_clusters.hpp>

#include <time.h>

int main (int argc, char** argv)
{
  // Read in the cloud data
  pcl::PCDReader reader;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCDWriter writer;
  reader.read ("/home/zb/BoZhan/PCL/table_scene_mug_stereo_textured_hull.pcd", *cloud_filtered);


/////////////////////////////////////////////
/// CPU VERSION
/////////////////////////////////////////////

  std::cout << "INFO: PointCloud_filtered still has " << cloud_filtered->points.size() << " Points " << std::endl;
  clock_t tStart = clock();
  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_filtered);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud( cloud_filtered);
  ec.extract (cluster_indices);

  printf("CPU Time taken: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);

  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    std::stringstream ss;
    ss << "cloud_cluster_" << j << ".pcd";
    writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
    j++;
  }



/////////////////////////////////////////////
/// GPU VERSION
/////////////////////////////////////////////

  std::cout << "INFO: starting with the GPU version" << std::endl;

  tStart = clock();

  pcl::gpu::Octree::PointCloud cloud_device;
  cloud_device.upload(cloud_filtered->points);

  pcl::gpu::Octree::Ptr octree_device (new pcl::gpu::Octree);
  octree_device->setCloud(cloud_device);
  octree_device->build();

  std::vector<pcl::PointIndices> cluster_indices_gpu;
  pcl::gpu::EuclideanClusterExtraction gec;
  gec.setClusterTolerance (0.02); // 2cm
  gec.setMinClusterSize (100);
  gec.setMaxClusterSize (25000);
  gec.setSearchMethod (octree_device);
  gec.setHostCloud( cloud_filtered);
  gec.extract (cluster_indices_gpu);
//  octree_device.clear();

  printf("GPU Time taken: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
  std::cout << "INFO: stopped with the GPU version" << std::endl;

  j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices_gpu.begin (); it != cluster_indices_gpu.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster_gpu (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster_gpu->points.push_back (cloud_filtered->points[*pit]); //*
    cloud_cluster_gpu->width = cloud_cluster_gpu->points.size ();
    cloud_cluster_gpu->height = 1;
    cloud_cluster_gpu->is_dense = true;

    std::cout << "PointCloud representing the Cluster: " << cloud_cluster_gpu->points.size () << " data points." << std::endl;
    std::stringstream ss;
    ss << "gpu_cloud_cluster_" << j << ".pcd";
    writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster_gpu, false); //*
    j++;
  }

  return (0);
}

