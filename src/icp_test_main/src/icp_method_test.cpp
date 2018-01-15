/************************************model and scene registration*************************************/
//author:Bo Zhan
//14,nov,2017

#include <iostream>
#include <string>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>
#include <pcl/registration/correspondence_estimation_normal_shooting.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/console/time.h>
#include <boost/make_shared.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/transforms.h>
#include <pcl/registration/super4pcs.h>
#include <super4pcs/shared4pcs.h>
#include <super4pcs/algorithms/4pcs.h>
#include <super4pcs/algorithms/super4pcs.h>
#include <super4pcs/io/io.h>
#include <super4pcs/utils/geometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/print.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/common/time.h>
#include <pcl/common/centroid.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/shot.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/features/narf_descriptor.h>
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/range_image_visualizer.h>

#include <boost/thread/thread.hpp>
#include <pcl/console/parse.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/correspondence.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/recognition/hv/hv_go.h>
#include <pcl/gpu/containers/device_array.h>
#include <pcl/gpu/features/features.hpp>
#include <time.h>







typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudNT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerT;

using namespace GlobalRegistration;
using namespace std;

bool next_iteration = false;



void print4x4Matrix (const Eigen::Matrix4d & matrix)
{
  printf ("Rotation matrix :\n");
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
  printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
  printf ("Translation vector :\n");
  printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent& event,void* nothing)
{
  if (event.getKeySym () == "space" && event.keyDown ())
    next_iteration = true;
}

double computeCloudResolution(const pcl::PointCloud<PointNT>::ConstPtr& cloud)
{
  double resolution = 0.0;
  int numberOfPoints = 0;
  int nres;
  std::vector<int> indices(2);
  std::vector<float> squaredDistances(2);
  pcl::search::KdTree<PointNT> tree;
  tree.setInputCloud(cloud);

  for (size_t i = 0; i < cloud->size(); ++i)
  {
    if (! pcl_isfinite((*cloud)[i].x))
      continue;

    // Considering the second neighbor since the first is the point itself.
    nres = tree.nearestKSearch(i, 2, indices, squaredDistances);
    if (nres == 2)
    {
      resolution += sqrt(squaredDistances[1]);
      ++numberOfPoints;
    }
  }
  if (numberOfPoints != 0)
    resolution /= numberOfPoints;

  return resolution;
}


int main (int argc,char* argv[])
{
  // The point clouds we will be using
  PointCloudNT::Ptr cloud_in (new PointCloudNT);  // Original point cloud
  PointCloudNT::Ptr cloud_tr (new PointCloudNT);  // Transformed point cloud
  PointCloudNT::Ptr cloud_icp (new PointCloudNT);  // ICP output point cloud
  PointCloudNT::Ptr cloud_in_nor_dis (new PointCloudNT);  // Original point cloud
  PointCloudNT::Ptr cloud_tr_nor_dis (new PointCloudNT);  // Transformed point cloud


  //downsample size
  const float leaf = 0.002f;
//    const float leaf = 0.0005f;
//  const float leaf = 0.002f;

  // Defining a rotation matrix and translation vector
  Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();

  //load model pointcloud
  pcl::console::TicToc time;
  time.tic ();
  PointCloudT::Ptr cloud_in_un (new PointCloudT);
  PointCloudT::Ptr cloud_in_un_copy (new PointCloudT);
  if (pcl::io::loadPCDFile<PointT>("/home/zb/BoZhan/picking_zb/model/model2.pcd", *cloud_in_un) == -1) //* 读入PCD格式的文件，如果文件不存在，返回-1
  {
      PCL_ERROR("Couldn't read file model.pcd \n"); //文件不存在时，返回错误，终止程序。
      return (-1);
  }
  *cloud_in_un_copy = *cloud_in_un;
  std::cout << "\nLoaded model file " << " (" << cloud_in_un->size() << " points) in " << time.toc () << " ms" << std::endl;
  std::cout << "point_height = "<<cloud_in_un->height<<"  "<<"point_width = "<<cloud_in_un->width<< std::endl;

  //remove NAN points from the cloud
  std::vector<int> indices_model;
  pcl::removeNaNFromPointCloud(*cloud_in_un,*cloud_in_un, indices_model);
  std::cout << "after remove NAN points " << cloud_in_un->size ()<<" points "  <<std::endl;

  //create the filtering object
  pcl::PCLPointCloud2::Ptr cloud_in_temp (new pcl::PCLPointCloud2 ());
  pcl::toPCLPointCloud2(*cloud_in_un,*cloud_in_temp);
  pcl::PCLPointCloud2::Ptr cloud_in_filtered (new pcl::PCLPointCloud2 ());
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud_in_temp);
  sor.setLeafSize (leaf, leaf, leaf);
  sor.filter (*cloud_in_filtered);
  pcl::fromPCLPointCloud2(*cloud_in_filtered,*cloud_in_un);
  std::cout << "after VoxelGrid downsample " << cloud_in_un->size ()<<" points "  <<std::endl;


//  std::cout<<*cloud_in_un<<std::endl;
  double x_max=0;
  double y_max=0;
  double z_max=0;
  for (size_t i = 0; i < cloud_in_un->size(); ++i)
  {
    double x_ =cloud_in_un->points[i].x;
    double y_ =cloud_in_un->points[i].y;
    double z_ =cloud_in_un->points[i].z;
    x_max = (x_max>x_)?x_max:x_;
    y_max = (y_max>y_)?y_max:y_;
    z_max = (z_max>z_)?z_max:z_;
//    cout<<cloud_in_un->points[i].z<<endl;
  }
  cout<<"x_max = "<<x_max<<endl;
  cout<<"y_max = "<<y_max<<endl;
  cout<<"z_max = "<<z_max<<endl;

  //  Compute Normals
  time.tic ();
  pcl::PointCloud<pcl::Normal>::Ptr model_normals (new pcl::PointCloud<pcl::Normal> ());
  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> norm_est_in;
  //  norm_est_in.setKSearch (10);
  norm_est_in.setRadiusSearch(5*leaf);
  norm_est_in.setInputCloud (cloud_in_un);
  norm_est_in.compute (*model_normals);
  std::cout<<"cpu normal compute cost time = "<<time.toc()<<std::endl;


//  //GPU
//  clock_t tStart = clock();
//  pcl::PointCloud<pcl::Normal>::Ptr model_normals_gpu (new pcl::PointCloud<pcl::Normal> ());
//  model_normals_gpu->width = cloud_in_un->width;
//  model_normals_gpu->height = cloud_in_un->height;
//  model_normals_gpu->resize(model_normals_gpu->width * model_normals_gpu->height);
////  std::cout<<"Gpu prepare cost time = "<<(double)(clock() - tStart)/CLOCKS_PER_SEC*1000<<std::endl;
////  tStart = clock();
//  pcl::gpu::Feature::PointCloud cloud_d_(cloud_in_un->width * cloud_in_un->height);
////  std::cout<<"Gpu create memory cost time = "<<(double)(clock() - tStart)/CLOCKS_PER_SEC*1000<<std::endl;
////  tStart = clock();
//  cloud_d_.upload(cloud_in_un->points);
////  std::cout<<"Gpu upload cost time = "<<(double)(clock() - tStart)/CLOCKS_PER_SEC*1000<<std::endl;
////  tStart = clock();
//  pcl::gpu::Feature::Normals normals_d(cloud_in_un->width * cloud_in_un->height);
////  std::cout<<"Gpu create memory time = "<<(double)(clock() - tStart)/CLOCKS_PER_SEC*1000<<std::endl;
////  tStart = clock();
//  pcl::gpu::NormalEstimation ne_d;
//  ne_d.setInputCloud(cloud_d_);
////  ne_d.setViewPoint(0, 0, 0);
//  ne_d.setRadiusSearch(0.01, 4);
//  ne_d.compute(normals_d);
////  std::cout<<"Gpu caculation cost time = "<<(double)(clock() - tStart)/CLOCKS_PER_SEC*1000<<std::endl;
////  tStart = clock();
//  normals_d.download((pcl::gpu::Octree::PointType*) &(model_normals_gpu->points[0]));
//  std::cout<<"Gpu download cost time = "<<(double)(clock() - tStart)/CLOCKS_PER_SEC*1000<<std::endl;

  //connect xyz and normal field
  pcl::concatenateFields(*cloud_in_un,*model_normals,*cloud_in);
  *cloud_in_nor_dis = *cloud_in;


  PointCloudNT::Ptr cloud_in_cp2 (new PointCloudNT);  // Original point cloud
  *cloud_in_cp2 = *cloud_in;

  //caculate pointcloud centroid and move frame origin to centroid
  Eigen::Vector4f centroid_in;
  pcl::compute3DCentroid(*cloud_in, centroid_in);
  for (size_t i = 0; i < cloud_in->size(); ++i)
  {
    cloud_in->points[i].x = cloud_in->points[i].x - centroid_in[0]*0;
    cloud_in->points[i].y = cloud_in->points[i].y - centroid_in[1]*0;
    cloud_in->points[i].z = cloud_in->points[i].z - centroid_in[2]*0;
  }
  std::cout<<"centroid_in transform = "<<centroid_in<<std::endl;

  Eigen::Matrix3d R_cin;
  Eigen::Vector3d t_cin;
  Eigen::Matrix4d transformation_matrix_cin = Eigen::Matrix4d::Identity ();
  R_cin = Eigen::Matrix3d::Identity ();
  t_cin[0] = centroid_in[0];
  t_cin[1] = centroid_in[1];
  t_cin[2] = centroid_in[2];
  transformation_matrix_cin.block(0,3,3,1) = t_cin;




//  // A rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
//  Eigen::Matrix4d transformation_matrix_icp_p = Eigen::Matrix4d::Identity ();
//  PointCloudT::Ptr cloud_icp_un (new PointCloudT);
//  double theta = 8.0*M_PI / 8;  // The angle of rotation in radians,z axis
//  transformation_matrix_icp_p (0, 0) = cos (theta);
//  transformation_matrix_icp_p (0, 1) = -sin (theta);
//  transformation_matrix_icp_p (1, 0) = sin (theta);
//  transformation_matrix_icp_p (1, 1) = cos (theta);

//  // A translation on Z axis (0.4 meters)
//  transformation_matrix_icp_p (0, 3) = 0.0;
//  transformation_matrix_icp_p (1, 3) = 0.0;
//  transformation_matrix_icp_p (2, 3) = 0.0;

//  // Display in terminal the transformation matrix
//  std::cout << "Applying this rigid transformation to: cloud_in -> cloud_icp" << std::endl;
//  print4x4Matrix (transformation_matrix_icp_p);

//  // Executing the transformation
//  std::cout << "cloud_in_un_copy = "<<cloud_in_un_copy->size()<< std::endl;
//  pcl::transformPointCloud (*cloud_in_un_copy, *cloud_icp_un, transformation_matrix_icp_p);



//  load scene pointcloud
  time.tic ();
  pcl::PCLPointCloud2::Ptr cloud_icp_temp (new pcl::PCLPointCloud2 ());
  PointCloudT::Ptr cloud_icp_un (new PointCloudT);
  if (pcl::io::loadPCDFile(/*"/home/zb/BoZhan/PCL/stand/r0.pcd"*/"/home/zb/BoZhan/picking_zb/devel/lib/picking/1/3.pcd", *cloud_icp_un) == -1) //* 读入PCD格式的文件，如果文件不存在，返回-1
  {
      PCL_ERROR("Couldn't read file source.pcd \n"); //文件不存在时，返回错误，终止程序。
      return (-1);
  }
  std::cout << "\nLoaded source file " << " (" << cloud_icp_un->size () << " points) in " << time.toc () << " ms" << std::endl;
  std::cout << "point_height = "<<cloud_icp_un->height<<"  "<<"point_width = "<<cloud_icp_un->width<< std::endl;
//  std::cout<<"corn points = "<<std::endl;
//  std::cout<<cloud_icp_un->at(127*cloud_icp_un->width+151)<<std::endl;
//  std::cout<<cloud_icp_un->at(120*cloud_icp_un->width+511)<<std::endl;
//  std::cout<<cloud_icp_un->at(380*cloud_icp_un->width+146)<<std::endl;
//  std::cout<<cloud_icp_un->at(120*cloud_icp_un->width+511)<<std::endl;

  //remove NAN points from the cloud
  std::vector<int> indices_source;
  pcl::removeNaNFromPointCloud(*cloud_icp_un,*cloud_icp_un, indices_source);
  std::cout << "after remove NAN points " << cloud_icp_un->size ()<<" points "  <<std::endl;

  //RadiusOutlierRemoval filter
  pcl::RadiusOutlierRemoval<PointT> outrem;
  outrem.setInputCloud(cloud_icp_un);
//  outrem.setRadiusSearch(leaf*10);
  outrem.setRadiusSearch(leaf*1000);
  outrem.setMinNeighborsInRadius (2);
  pcl::PointCloud<PointT>::Ptr cloud_icp_rad_rem (new pcl::PointCloud<PointT>);
  outrem.filter (*cloud_icp_rad_rem);
  std::cout << "after RadiusOutlierRemoval " << cloud_icp_rad_rem->size()<<" points "  <<std::endl;

  //VoxelGrid downsample
  pcl::toPCLPointCloud2(*cloud_icp_rad_rem,*cloud_icp_temp);
  pcl::PCLPointCloud2::Ptr cloud_icp_filtered (new pcl::PCLPointCloud2 ());
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor2;
  sor2.setInputCloud (cloud_icp_temp);
//  sor2.setLeafSize (leaf, leaf, leaf);
  sor2.setLeafSize (0.01*leaf, 0.01*leaf, 0.01*leaf);
  sor2.filter (*cloud_icp_filtered);
  pcl::fromPCLPointCloud2(*cloud_icp_filtered,*cloud_icp_rad_rem);
  std::cout << "after VoxelGrid downsample " << cloud_icp_rad_rem->size ()<<" points\n"  <<std::endl;


  //  Compute Normals
  pcl::PointCloud<pcl::Normal>::Ptr source_normals (new pcl::PointCloud<pcl::Normal> ());
  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> norm_est;
  //  norm_est.setKSearch (10);
  norm_est.setRadiusSearch(5*leaf);
  //  norm_est.setViewPoint(0,0,0);
  norm_est.setInputCloud (cloud_icp_rad_rem);
  norm_est.compute (*source_normals);

  //connect xyz and normal
  pcl::concatenateFields(*cloud_icp_rad_rem,*source_normals,*cloud_icp);
  *cloud_tr_nor_dis = *cloud_icp;

  // backup cloud_icp into cloud_tr for later visulation
  *cloud_tr = *cloud_icp;

  //caculate pointcloud centroid and move frame origin to centroid
  Eigen::Vector4f centroid_icp;
  pcl::compute3DCentroid(*cloud_icp, centroid_icp);
  for (size_t i = 0; i < cloud_icp->size(); ++i)
  {
    cloud_icp->points[i].x = cloud_icp->points[i].x - centroid_icp[0];
    cloud_icp->points[i].y = cloud_icp->points[i].y - centroid_icp[1];
    cloud_icp->points[i].z = cloud_icp->points[i].z - centroid_icp[2];
  }
  std::cout<<"centroid_icp transform = "<<centroid_icp<<std::endl;

  Eigen::Matrix3d R_cicp;
  Eigen::Vector3d t_cicp;
  Eigen::Matrix4d transformation_matrix_cicp = Eigen::Matrix4d::Identity ();
  R_cicp = Eigen::Matrix3d::Identity ();
  t_cicp[0] = centroid_icp[0];
  t_cicp[1] = centroid_icp[1];
  t_cicp[2] = centroid_icp[2];
  transformation_matrix_cicp.block(0,3,3,1) = t_cicp;




  // Visualization
  pcl::visualization::PCLVisualizer viewer ("ICP demo");
  int v1 (0);
  int v2 (1);
  viewer.createViewPort (0.0, 0.0, 0.5, 1.0, v1);
  viewer.createViewPort (0.5, 0.0, 1.0, 1.0, v2);
  float bckgr_gray_level = 0.0;  // Black
  float txt_gray_lvl = 1.0 - bckgr_gray_level;
  pcl::visualization::PointCloudColorHandlerCustom<PointNT> cloud_in_color_h (cloud_in, (int) 255 * txt_gray_lvl, (int) 255 * txt_gray_lvl,
                                                                             (int) 255 * txt_gray_lvl);
  viewer.addPointCloud (cloud_in, cloud_in_color_h, "cloud_in_v1", v1);
  const std::string frame_id = "camera";
  viewer.addCoordinateSystem(0.01,frame_id,v1);
  viewer.addPointCloud (cloud_in, cloud_in_color_h, "cloud_in_v2", v2);
  //  viewer.addPointCloudNormals<PointNT, pcl::Normal>(cloud_in_nor_dis, model_normals, 10, 0.03, "in_normals",v1);
  pcl::visualization::PointCloudColorHandlerCustom<PointNT> cloud_tr_color_h (cloud_tr, 20, 180, 20);
  viewer.addPointCloud (cloud_tr, cloud_tr_color_h, "cloud_tr_v1", v1);
  //  viewer.addPointCloudNormals<PointNT, pcl::Normal>(cloud_tr_nor_dis, source_normals, 10, 0.03, "tr_normals",v1);
  pcl::visualization::PointCloudColorHandlerCustom<PointNT> cloud_icp_color_h (cloud_icp, 180, 20, 20);
  viewer.addPointCloud (cloud_icp, cloud_icp_color_h, "cloud_icp_v2", v2);
  viewer.addCoordinateSystem(0.01,frame_id,v2);
  viewer.addText ("White: Original point cloud\nGreen: Matrix transformed point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v1);
  viewer.addText ("White: Original point cloud\nRed: ICP aligned point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_2", v2);
  viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
  viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v2);
  viewer.setCameraPosition (-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
  viewer.setSize (1280, 1024);
  viewer.registerKeyboardCallback (&keyboardEventOccurred, (void*) NULL);


  /*****************************************registration algorithm*********************************************/

  /**
   * method 1:super_4pcs_based registration*
  */
  time.tic ();

  //coarse alignment(super_4PCS)
  pcl::Super4PCS<PointNT,PointNT> icp_4pcs;
  pcl::console::print_highlight ("Starting alignment...\n");
  icp_4pcs.setInputSource (cloud_icp);
  icp_4pcs.setInputTarget (cloud_in);
  icp_4pcs.options_.sample_size = 50;//100
  icp_4pcs.options_.max_normal_difference = -1.00;//-1.00
  icp_4pcs.options_.max_color_distance = -1.00;//-1.00
  icp_4pcs.options_.max_time_seconds = 10000;//10000
  icp_4pcs.options_.delta = 0.01;//0.01
  icp_4pcs.options_.configureOverlap(0.9);//0.9
  icp_4pcs.align (*cloud_icp);
  std::cout << "super_4pcs cost  " << time.toc () << " ms" << std::endl;
  Eigen::Matrix4d transformation_matrix_4pcs = Eigen::Matrix4d::Identity ();
  if (icp_4pcs.hasConverged ())
  {
  //    std::cout << "\nICP has converged, score is " << icp.getFitnessScore () << std::endl;
    std::cout << "\n4pcs_ICP transformation " << " : cloud_icp -> cloud_in" << std::endl;
    transformation_matrix_4pcs = icp_4pcs.getFinalTransformation ().cast<double>();
//    print4x4Matrix (transformation_matrix_4pcs);
    viewer.updatePointCloud (cloud_icp, cloud_icp_color_h, "cloud_icp_v2");
  }
  else
  {
    PCL_ERROR ("\nICP has not converged.\n");
    return (-1);
  }

  ////apply ICP refine
  pcl::IterativeClosestPointWithNormals<PointNT, PointNT> icp;
  int iterations = 100;  // Default number of ICP iterations
  icp.setMaximumIterations (iterations);
  icp.setTransformationEpsilon (1e-6);
  icp.setMaxCorrespondenceDistance (0.01);  // Set the maximum distance between two correspondences (src<->tgt) to 10cm
  icp.setInputSource (cloud_icp);
  icp.setInputTarget (cloud_in);
  Eigen::Matrix4d transformation_matrix_icp = Eigen::Matrix4d::Identity ();
  while (!viewer.wasStopped ())
  {
    viewer.spinOnce ();
    // The user pressed "space" :
    if (next_iteration)
    {
      time.tic ();
      icp.align (*cloud_icp);
      std::cout << "ICP refine cost " << time.toc () << " ms" << std::endl;
      break;
    }
  }
  if (icp.hasConverged ())
  {
    printf ("\nICP has converged, score is %+.0e\n", icp.getFitnessScore ());
    std::cout << "\nICP transformation: cloud_icp -> cloud_in" << std::endl;
    transformation_matrix_icp = icp.getFinalTransformation ().cast<double>();
//    print4x4Matrix (transformation_matrix_icp);  // Print the transformation between original pose and current pose
    viewer.updatePointCloud (cloud_icp, cloud_icp_color_h, "cloud_icp_v2");
  }
  else
  {
    PCL_ERROR ("\nICP has not converged.\n");
    return (-1);
  }

  transformation_matrix = transformation_matrix_cin*transformation_matrix_icp*transformation_matrix_4pcs*transformation_matrix_cicp.inverse();
  printf("Tranform: cloud_icp frame in cloud_in frame\n");

  Eigen::Matrix4d transformation_matrix_inv = transformation_matrix.inverse();
  print4x4Matrix (transformation_matrix_inv);
  PointCloudNT::Ptr cloud_in_cp2_cp (new PointCloudNT);  // Original point cloud
  pcl::transformPointCloud (*cloud_in_cp2, *cloud_in_cp2_cp,transformation_matrix_inv );
  viewer.addPointCloud (cloud_in_cp2_cp, cloud_in_color_h, "cloud_in_cp_v2", v1);



//  /**
//   * method 2:feature_based registration*
//  */

//  ////coarse alignment
//  pcl::console::print_highlight ("Estimating features...\n");
//  ////find keypoints
//  /// keypoint detector ISS
//  pcl::PointCloud<PointNT>::Ptr keypoints_in(new pcl::PointCloud<PointNT>);
//  pcl::ISSKeypoint3D<PointNT, PointNT> detector_in;
//  detector_in.setInputCloud(cloud_in);
//  pcl::search::KdTree<PointNT>::Ptr kdtree_in(new pcl::search::KdTree<PointNT>);
//  detector_in.setSearchMethod(kdtree_in);
//  double resolution_in = computeCloudResolution(cloud_in);
//  detector_in.setSalientRadius(6 * resolution_in);  // Set the radius of the spherical neighborhood used to compute the scatter matrix.
//  detector_in.setNonMaxRadius(4 * resolution_in);  // Set the radius for the application of the non maxima supression algorithm.
//  detector_in.setMinNeighbors(5);  // Set the minimum number of neighbors that has to be found while applying the non maxima suppression algorithm.
//  detector_in.setThreshold21(0.975);  // Set the upper bound on the ratio between the second and the first eigenvalue.
//  detector_in.setThreshold32(0.975);  // Set the upper bound on the ratio between the third and the second eigenvalue.
//  detector_in.setNumberOfThreads(4);  // Set the number of prpcessing threads to use. 0 sets it to automatic.
//  detector_in.compute(*keypoints_in);

//  pcl::PointCloud<PointNT>::Ptr keypoints_icp(new pcl::PointCloud<PointNT>);
//  pcl::ISSKeypoint3D<PointNT, PointNT> detector_icp;
//  detector_icp.setInputCloud(cloud_icp);
//  pcl::search::KdTree<PointNT>::Ptr kdtree_icp(new pcl::search::KdTree<PointNT>);
//  detector_icp.setSearchMethod(kdtree_icp);
//  double resolution_icp = computeCloudResolution(cloud_icp);
//  detector_icp.setSalientRadius(6 * resolution_icp);  // Set the radius of the spherical neighborhood used to compute the scatter matrix.
//  detector_icp.setNonMaxRadius(4 * resolution_icp);  // Set the radius for the application of the non maxima supression algorithm.
//  detector_icp.setMinNeighbors(5);  // Set the minimum number of neighbors that has to be found while applying the non maxima suppression algorithm.
//  detector_icp.setThreshold21(0.975);  // Set the upper bound on the ratio between the second and the first eigenvalue.
//  detector_icp.setThreshold32(0.975);  // Set the upper bound on the ratio between the third and the second eigenvalue.
//  detector_icp.setNumberOfThreads(4);  // Set the number of prpcessing threads to use. 0 sets it to automatic.
//  detector_icp.compute(*keypoints_icp);

//  //display keypoints
////  pcl::visualization::PointCloudColorHandlerCustom<PointNT> keypoint_in_color_h (keypoints_in, 255 ,0,0);
////  viewer.addPointCloud (keypoints_in, keypoint_in_color_h, "keypoint_in_v1", v1);
////  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5,"keypoint_in_v1", v1);
////  pcl::visualization::PointCloudColorHandlerCustom<PointNT> keypoint_icp_color_h (keypoints_icp, 0 ,0,255);
////  viewer.addPointCloud (keypoints_icp, keypoint_icp_color_h, "keypoint_icp_v1", v2);
////  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "keypoint_icp_v1", v2);



//  ////compute descriptor
//  // FPFH descriptor
////  pcl::PointCloud<pcl::FPFHSignature33>::Ptr cloud_in_features (new pcl::PointCloud<pcl::FPFHSignature33>);
////  pcl::PointCloud<pcl::FPFHSignature33>::Ptr cloud_icp_features (new pcl::PointCloud<pcl::FPFHSignature33>);
////  pcl::FPFHEstimationOMP<PointNT,PointNT,pcl::FPFHSignature33> fest;
////  fest.setRadiusSearch (5.5*leaf);
////  fest.setInputCloud (cloud_in);
////  fest.setInputNormals (cloud_in);
////  fest.compute (*cloud_in_features);
////  fest.setInputCloud (cloud_icp);
////  fest.setInputNormals (cloud_icp);
////  fest.compute (*cloud_icp_features);

////  // SHOT estimation object.
//  pcl::PointCloud<pcl::SHOT352>::Ptr cloud_in_features(new pcl::PointCloud<pcl::SHOT352>());
//  pcl::PointCloud<pcl::SHOT352>::Ptr cloud_icp_features(new pcl::PointCloud<pcl::SHOT352>());
//  pcl::SHOTEstimationOMP<PointNT, PointNT, pcl::SHOT352> shot;
//  // The radius that defines which of the keypoint's neighbors are described.
//  // If too large, there may be clutter, and if too small, not enough points may be found.
//  shot.setRadiusSearch (50.5*leaf);
//  shot.setInputCloud (keypoints_in);
//  shot.setInputNormals (keypoints_in);
//  shot.compute (*cloud_in_features);
//  shot.setInputCloud (keypoints_icp);
//  shot.setInputNormals (keypoints_icp);
//  shot.compute (*cloud_icp_features);


//  //match
//  pcl::KdTreeFLANN<pcl::SHOT352> matching;
//  matching.setInputCloud(cloud_in_features);
//  pcl::CorrespondencesPtr correspondences(new pcl::Correspondences());
//  for (size_t i = 0; i < cloud_icp_features->size(); ++i)
//  {
//    std::vector<int> neighbors(1);
//    std::vector<float> squaredDistances(1);
//    // Ignore NaNs.
//    if (pcl_isfinite(cloud_icp_features->at(i).descriptor[0]))
//    {
//      int neighborCount = matching.nearestKSearch(cloud_icp_features->at(i), 1, neighbors, squaredDistances);
//      // ...and add a new correspondence if the distance is less than a threshold
//      // (SHOT distances are between 0 and 1, other descriptors use different metrics).
//      if (neighborCount == 1 /*&& squaredDistances[0] < 3.0f*/)
//      {
//        pcl::Correspondence correspondence(neighbors[0], static_cast<int>(i), squaredDistances[0]);
//        correspondences->push_back(correspondence);
//      }
//    }
//  }
//  std::cout << "Found " << correspondences->size() << " correspondences." << std::endl;



//  ////Correspondence grouping
//  std::vector<pcl::Correspondences> clusteredCorrespondences;
//  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > transformations_cor;
//  pcl::GeometricConsistencyGrouping<PointNT, PointNT> grouping;
//  grouping.setInputCloud(keypoints_in);
//  grouping.setSceneCloud(keypoints_icp);
//  grouping.setModelSceneCorrespondences(correspondences);
//  // Minimum cluster size. Default is 3 (as at least 3 correspondences
//  // are needed to compute the 6 DoF pose).
//  grouping.setGCThreshold(3);
//  // Resolution of the consensus set used to cluster correspondences together,
//  // in metric units. Default is 1.0.
//  grouping.setGCSize(0.02);
//  grouping.recognize(transformations_cor, clusteredCorrespondences);
//  std::cout << "Model instances found: " << transformations_cor.size() << std::endl << std::endl;
//  for (size_t i = 0; i < transformations_cor.size(); i++)
//  {
//    std::cout << "Instance " << (i + 1) << ":" << std::endl;
//    std::cout << "\tHas " << clusteredCorrespondences[i].size() << " correspondences." << std::endl << std::endl;
//    Eigen::Matrix3f rotation_cor = transformations_cor[i].block<3, 3>(0, 0);
//    Eigen::Vector3f translation_cor = transformations_cor[i].block<3, 1>(0, 3);
//    printf("\t\t    | %6.3f %6.3f %6.3f | \n", rotation_cor(0, 0), rotation_cor(0, 1), rotation_cor(0, 2));
//    printf("\t\tR = | %6.3f %6.3f %6.3f | \n", rotation_cor(1, 0), rotation_cor(1, 1), rotation_cor(1, 2));
//    printf("\t\t    | %6.3f %6.3f %6.3f | \n", rotation_cor(2, 0), rotation_cor(2, 1), rotation_cor(2, 2));
//    std::cout << std::endl;
//    printf("\t\tt = < %0.3f, %0.3f, %0.3f >\n", translation_cor(0), translation_cor(1), translation_cor(2));
//  }

//   //// exact xyz field and discard normal field
//  PointCloudT::Ptr cloud_in_narf (new PointCloudT);  // Original point cloud
//  PointCloudT::Ptr cloud_icp_narf (new PointCloudT);  // ICP output point cloud
//  cloud_in_narf->resize(cloud_in->size());
//  cloud_icp_narf->resize(cloud_icp->size());
//  cloud_in_narf->height = cloud_in->height;
//  cloud_in_narf->width = cloud_in->width;
//  cloud_icp_narf->height = cloud_icp->height;
//  cloud_icp_narf->width = cloud_icp->width;
//  for(size_t i=0;i<cloud_in->height;i++)
//  {
//    for(size_t j=0;j<cloud_in->width;j++)
//    {
//        cloud_in_narf->points[i*cloud_in->width+j].x = cloud_in->points[i*cloud_in->width+j].x;
//        cloud_in_narf->points[i*cloud_in->width+j].y = cloud_in->points[i*cloud_in->width+j].y;
//        cloud_in_narf->points[i*cloud_in->width+j].z = cloud_in->points[i*cloud_in->width+j].z;
//    }
//  }

//  for(size_t i=0;i<cloud_icp->height;i++)
//  {
//    for(size_t j=0;j<cloud_icp->width;j++)
//    {
//        cloud_icp_narf->points[i*cloud_icp->width+j].x = cloud_icp->points[i*cloud_icp->width+j].x;
//        cloud_icp_narf->points[i*cloud_icp->width+j].y = cloud_icp->points[i*cloud_icp->width+j].y;
//        cloud_icp_narf->points[i*cloud_icp->width+j].z = cloud_icp->points[i*cloud_icp->width+j].z;
//    }
//  }


//  ///rotate model
//  std::vector<pcl::PointCloud<PointT>::ConstPtr> instances;
//  for (size_t i = 0; i < transformations_cor.size (); ++i)
//  {
//    pcl::PointCloud<PointT>::Ptr rotated_model (new pcl::PointCloud<PointT> ());
//    pcl::transformPointCloud (*cloud_in_narf, *rotated_model, transformations_cor[i]);
//    instances.push_back (rotated_model);
//  }

//  ///ICP refine
//  std::vector<pcl::PointCloud<PointT>::ConstPtr> registered_instances;
//  if (true)
//  {
//    cout << "--- ICP ---------" << endl;
//    for (size_t i = 0; i < transformations_cor.size (); ++i)
//    {
//      pcl::IterativeClosestPoint<PointT, PointT> icp_hv;
//      icp_hv.setMaximumIterations (10);
//      icp_hv.setMaxCorrespondenceDistance (0.005f);
//      icp_hv.setInputSource (instances[i]);
//      icp_hv.setInputTarget (cloud_icp_narf);
//      pcl::PointCloud<PointT>::Ptr registered (new pcl::PointCloud<PointT>);
//      icp_hv.align (*registered);
//      registered_instances.push_back (registered);
//      cout << "Instance " << i << " ";
//      if (icp_hv.hasConverged ())
//      {
//        cout << "Aligned!" << endl;
//      }
//      else
//      {
//        cout << "Not Aligned!" << endl;
//      }
//    }

//    cout << "-----------------" << endl << endl;
//  }


//  ////Hypothesis Verification
//  cout << "--- Hypotheses Verification ---" << endl;
//  std::vector<bool> hypotheses_mask;  // Mask Vector to identify positive hypotheses
//  pcl::GlobalHypothesesVerification<PointT, PointT> GoHv;
//  GoHv.setSceneCloud (cloud_icp_narf);  // Scene Cloud
//  GoHv.addModels (registered_instances, true);  //Models to verify
//  GoHv.setInlierThreshold (0.05f);
//  GoHv.setResolution(0.001f);
//  GoHv.setOcclusionThreshold (0.03f);
//  GoHv.setRegularizer (3.0f);//outliers' model weight
//  GoHv.setRadiusClutter (0.05);
//  GoHv.setClutterRegularizer (5.0f);//clutter points weight
//  GoHv.setDetectClutter (true);
//  GoHv.setRadiusNormals (0.05);
//  GoHv.verify();
//  GoHv.getMask (hypotheses_mask);  // i-element TRUE if hvModels[i] verifies hypotheses
//  for (int i = 0; i < hypotheses_mask.size (); i++)
//  {
//    if (hypotheses_mask[i])
//    {
//      cout << "Instance " << i+1 << " is GOOD! <---" << endl;
//    }
//    else
//    {
//      cout << "Instance " << i+1 << " is bad!" << endl;
//    }
//  }
//  cout << "-------------------------------" << endl;

//  ////  Compute Normals and generate pointNormal
//  std::vector<pcl::PointCloud<PointNT>::ConstPtr> instances_nor;
//  pcl::PointCloud<pcl::Normal>::Ptr source_normals_HV (new pcl::PointCloud<pcl::Normal> ());
//  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> norm_est_hv;
//  //  norm_est.setKSearch (10);
//  norm_est_hv.setRadiusSearch(5*leaf);
//  //  norm_est.setViewPoint(0,0,0);
//  for(size_t i=0;i<instances.size();i++)
//  {
//    norm_est_hv.setInputCloud (instances[i]);
//    norm_est_hv.compute (*source_normals_HV);
//    pcl::PointCloud<PointNT>::Ptr instances_nor_one (new pcl::PointCloud<PointNT> ());
//    pcl::concatenateFields(*instances[i],*source_normals_HV,*instances_nor_one);
//    instances_nor.push_back(instances_nor_one);
//  }

//  std::vector<pcl::PointCloud<PointNT>::ConstPtr> instances_reg_nor;
//  pcl::PointCloud<pcl::Normal>::Ptr source_normals_reg_HV (new pcl::PointCloud<pcl::Normal> ());
//  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> norm_est_reg_hv;
//  //  norm_est.setKSearch (10);
//  norm_est_reg_hv.setRadiusSearch(5*leaf);
//  //  norm_est.setViewPoint(0,0,0);
//  for(size_t i=0;i<registered_instances.size();i++)
//  {
//    norm_est_hv.setInputCloud (registered_instances[i]);
//    norm_est_hv.compute (*source_normals_reg_HV);
//    pcl::PointCloud<PointNT>::Ptr instances_nor_reg_one (new pcl::PointCloud<PointNT> ());
//    pcl::concatenateFields(*registered_instances[i],*source_normals_reg_HV,*instances_nor_reg_one);
//    instances_reg_nor.push_back(instances_nor_reg_one);
//  }

//  ////Visualization
//  for (size_t i = 0; i < instances_nor.size (); ++i)
//  {
//    std::stringstream ss_instance;
//    ss_instance << "instance_" << i;
//    double r_,g_,b_;
//    ss_instance << "_registered" << endl;
//    if(hypotheses_mask[i])
//    {
//      r_ = 0.0;
//      g_ = 255;
//      b_ = 0;
////    }
////    else
////    {
////      r_ = 255;
////      g_ = 0;
////      b_ = 0;
////    }
//    pcl::visualization::PointCloudColorHandlerCustom<PointNT> registered_instance_color_handler (instances_reg_nor[i], r_,g_, b_);
//    viewer.addPointCloud (instances_reg_nor[i], registered_instance_color_handler, ss_instance.str (),v2);
//    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, ss_instance.str ());
//    }
//  }

  /************************************the end of registration algorithm*****************************************/

  while (!viewer.wasStopped ())
  {
    viewer.spinOnce ();
  }
  return (0);
}


//  // Perform alignment(Pose estimation)
//  pcl::console::print_highlight ("Starting alignment...\n");
//  PointCloudNT::Ptr object_aligned (new PointCloudNT);
//  pcl::SampleConsensusPrerejective<PointNT,PointNT,pcl::FPFHSignature33> align;
////  pcl::SampleConsensusPrerejective<PointNT,PointNT,pcl::SHOT352> align;
//  align.setInputSource (cloud_icp);
//  align.setSourceFeatures (cloud_icp_features);
//  align.setInputTarget (cloud_in);
//  align.setTargetFeatures (cloud_in_features);
//  align.setMaximumIterations (50000); // Number of RANSAC iterations
//  align.setNumberOfSamples (3); // Number of points to sample for generating/prerejecting a pose
//  align.setCorrespondenceRandomness (5); // Number of nearest features to use
//  align.setSimilarityThreshold (0.5f); // Polygonal edge length similarity threshold
//  double max_correspondence_distance_ = 5.5f * leaf;
//  align.setMaxCorrespondenceDistance (max_correspondence_distance_); // Inlier threshold
//  align.setInlierFraction (0.3f); // Required inlier fraction for accepting a pose hypothesis
//  // Display the visualiser
//  while (!viewer.wasStopped ())
//  {
//    viewer.spinOnce ();
//    // The user pressed "space" :
//    if (next_iteration)
//    {
//      pcl::ScopeTime t("Alignment");
//      align.align (*object_aligned);
//      next_iteration = false;
//      break;
//    }
//  }
//  *cloud_icp = *object_aligned;
//  viewer.updatePointCloud (cloud_icp, cloud_icp_color_h, "cloud_icp_v2");
//  if (align.hasConverged ())
//  {
//    // Print results
//    printf ("\n");
//    float fitness_score;
//    fitness_score = (float) align.getFitnessScore (max_correspondence_distance_);
//    Eigen::Matrix4f transformation_align = align.getFinalTransformation ();
//    pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation_align (0,0), transformation_align (0,1), transformation_align (0,2));
//    pcl::console::print_info ("R = | %6.3f %6.3f %6.3f | \n", transformation_align (1,0), transformation_align (1,1), transformation_align (1,2));
//    pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation_align (2,0), transformation_align (2,1), transformation_align (2,2));
//    pcl::console::print_info ("\n");
//    pcl::console::print_info ("t = < %0.3f, %0.3f, %0.3f >\n", transformation_align (0,3), transformation_align (1,3), transformation_align (2,3));
//    pcl::console::print_info ("\n");
//    pcl::console::print_info ("Inliers: %i/%i\n", align.getInliers ().size (), cloud_in->size ());
//  }
//  else
//  {
//    pcl::console::print_error ("Alignment failed!\n");
//  }



  // Display the visualiser
//  while (!viewer.wasStopped ())
//  {
//    viewer.spinOnce ();

//    // The user pressed "space" :
//    if (next_iteration)
//    {
//      // The Iterative Closest Point algorithm
//      time.tic ();
//      icp.align (*cloud_icp);
//      std::cout << "Applied 1 ICP iteration in " << time.toc () << " ms" << std::endl;

//      if (icp.hasConverged ())
//      {
////        printf ("\033[11A");  // Go up 11 lines in terminal output.
//        printf ("\nICP has converged, score is %+.0e\n", icp.getFitnessScore ());
//        std::cout << "\nICP transformation " << ++iterations << " : cloud_icp -> cloud_in" << std::endl;
//        transformation_matrix *= icp.getFinalTransformation ().cast<double>();  // WARNING /!\ This is not accurate! For "educational" purpose only!
//        print4x4Matrix (transformation_matrix);  // Print the transformation between original pose and current pose

//        ss.str ("");
//        ss << iterations;
//        std::string iterations_cnt = "ICP iterations = " + ss.str ();
//        viewer.updateText (iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt");
//        viewer.updatePointCloud (cloud_icp, cloud_icp_color_h, "cloud_icp_v2");
////        viewer.updatePointCloud (cloud_tr, cloud_tr_color_h, "cloud_tr_v1");
//      }
//      else
//      {
//        PCL_ERROR ("\nICP has not converged.\n");
//        return (-1);
//      }
//    }
//    next_iteration = false;
//  }
//  return (0);
//}



/***********************************copy point cloud******************************************/
//  PointCloudT::Ptr cloud_in_narf (new PointCloudT);  // Original point cloud
//  PointCloudT::Ptr cloud_icp_narf (new PointCloudT);  // ICP output point cloud
//  cloud_in_narf->resize(cloud_in->size());
//  cloud_icp_narf->resize(cloud_icp->size());
//  cloud_in_narf->height = cloud_in->height;
//  cloud_in_narf->width = cloud_in->width;
//  cloud_icp_narf->height = cloud_icp->height;
//  cloud_icp_narf->width = cloud_icp->width;
//  for(size_t i=0;i<cloud_in->height;i++)
//  {
//    for(size_t j=0;j<cloud_in->width;j++)
//    {
//        cloud_in_narf->points[i*cloud_in->width+j].x = cloud_in->points[i*cloud_in->width+j].x;
//        cloud_in_narf->points[i*cloud_in->width+j].y = cloud_in->points[i*cloud_in->width+j].y;
//        cloud_in_narf->points[i*cloud_in->width+j].z = cloud_in->points[i*cloud_in->width+j].z;
//    }
//  }

//  for(size_t i=0;i<cloud_icp->height;i++)
//  {
//    for(size_t j=0;j<cloud_icp->width;j++)
//    {
//        cloud_icp_narf->points[i*cloud_icp->width+j].x = cloud_icp->points[i*cloud_in->width+j].x;
//        cloud_icp_narf->points[i*cloud_icp->width+j].y = cloud_icp->points[i*cloud_in->width+j].y;
//        cloud_icp_narf->points[i*cloud_icp->width+j].z = cloud_icp->points[i*cloud_in->width+j].z;
//    }
//  }
/*********************************************************************************/



/************************************transform************************************/
//Eigen::Matrix3d R_,R_inv;
//Eigen::Vector3d t_,t_inv;
//R_ = transformation_matrix.block(0,0,3,3);
//t_ = transformation_matrix.block(0,3,3,1);
//R_inv = R_.transpose();
//t_inv = -R_.transpose()*t_;
//Eigen::Matrix4d transformation_matrix_inv=Eigen::Matrix4d::Identity ();
//transformation_matrix_inv.block(0,0,3,3) = R_inv;
//transformation_matrix_inv.block(0,3,3,1) = t_inv;
//pcl::transformPointCloud (*cloud_tr, *cloud_tr, transformation_matrix);
/********************************************************************************/


/**************************************several icp methods *************************************/
//////apply ICP refine
////  pcl::IterativeClosestPoint<PointNT, PointNT> icp;
////  pcl::IterativeClosestPointNonLinear<PointNT, PointNT> icp;
//pcl::IterativeClosestPointWithNormals<PointNT, PointNT> icp;
//int iterations = 10000;  // Default number of ICP iterations
//icp.setMaximumIterations (iterations);
//icp.setTransformationEpsilon (1e-6);
//icp.setMaxCorrespondenceDistance (0.01);  // Set the maximum distance between two correspondences (src<->tgt) to 10cm
////  pcl::registration::CorrespondenceEstimationNormalShooting<PointNT, PointNT, PointNT>::Ptr cens (new pcl::registration::CorrespondenceEstimationNormalShooting<PointNT, PointNT, PointNT>);
////  cens->setInputSource (cloud_icp);
////  cens->setInputTarget (cloud_in);
////  cens->setSourceNormals (cloud_icp);
////  icp.setCorrespondenceEstimation (cens);
////  pcl::registration::TransformationEstimationPointToPlaneLLS<PointNT, PointNT>::Ptr trans_lls (new pcl::registration::TransformationEstimationPointToPlaneLLS<PointNT, PointNT>);
////  icp.setTransformationEstimation (trans_lls);

//icp.setInputSource (cloud_icp);
//icp.setInputTarget (cloud_in);

////  // Initializing Normal Distributions Transform (NDT).
////  pcl::NormalDistributionsTransform<PointNT, PointNT> icp;
////  icp.setTransformationEpsilon (0.0001);
////  icp.setStepSize (0.002);
////  icp.setResolution (0.002);
////  icp.setMaximumIterations (35);
////  icp.setInputSource (cloud_icp);
////  icp.setInputTarget (cloud_in);
////  icp.align (*cloud_icp);
/*****************************************************************************/



/************************************example 2*************************************/
//#include <boost/make_shared.hpp>
//#include <pcl/point_types.h>
//#include <pcl/point_cloud.h>
//#include <pcl/point_representation.h>

//#include <pcl/io/pcd_io.h>

//#include <pcl/filters/voxel_grid.h>
//#include <pcl/filters/filter.h>

//#include <pcl/features/normal_3d.h>

//#include <pcl/registration/icp.h>
//#include <pcl/registration/icp_nl.h>
//#include <pcl/registration/transforms.h>

//#include <pcl/visualization/pcl_visualizer.h>

//using pcl::visualization::PointCloudColorHandlerGenericField;
//using pcl::visualization::PointCloudColorHandlerCustom;

////convenient typedefs
//typedef pcl::PointXYZ PointT;
//typedef pcl::PointCloud<PointT> PointCloud;
//typedef pcl::PointNormal PointNormalT;
//typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

//// This is a tutorial so we can afford having global variables
//  //our visualizer
//  pcl::visualization::PCLVisualizer *p;
//  //its left and right viewports
//  int vp_1, vp_2;

////convenient structure to handle our pointclouds
//struct PCD
//{
//  PointCloud::Ptr cloud;
//  std::string f_name;

//  PCD() : cloud (new PointCloud) {};
//};

//struct PCDComparator
//{
//  bool operator () (const PCD& p1, const PCD& p2)
//  {
//    return (p1.f_name < p2.f_name);
//  }
//};


//// Define a new point representation for < x, y, z, curvature >
//class MyPointRepresentation : public pcl::PointRepresentation <PointNormalT>
//{
//  using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
//public:
//  MyPointRepresentation ()
//  {
//    // Define the number of dimensions
//    nr_dimensions_ = 4;
//  }

//  // Override the copyToFloatArray method to define our feature vector
//  virtual void copyToFloatArray (const PointNormalT &p, float * out) const
//  {
//    // < x, y, z, curvature >
//    out[0] = p.x;
//    out[1] = p.y;
//    out[2] = p.z;
//    out[3] = p.curvature;
//  }
//};


//////////////////////////////////////////////////////////////////////////////////
///** \brief Display source and target on the first viewport of the visualizer
// *
// */
//void showCloudsLeft(const PointCloud::Ptr cloud_target, const PointCloud::Ptr cloud_source)
//{
//  p->removePointCloud ("vp1_target");
//  p->removePointCloud ("vp1_source");

//  PointCloudColorHandlerCustom<PointT> tgt_h (cloud_target, 0, 255, 0);
//  PointCloudColorHandlerCustom<PointT> src_h (cloud_source, 255, 0, 0);
//  p->addPointCloud (cloud_target, tgt_h, "vp1_target", vp_1);
//  p->addPointCloud (cloud_source, src_h, "vp1_source", vp_1);

//  PCL_INFO ("Press q to begin the registration.\n");
//  p-> spin();
//}


//////////////////////////////////////////////////////////////////////////////////
///** \brief Display source and target on the second viewport of the visualizer
// *
// */
//void showCloudsRight(const PointCloudWithNormals::Ptr cloud_target, const PointCloudWithNormals::Ptr cloud_source)
//{
//  p->removePointCloud ("source");
//  p->removePointCloud ("target");


//  PointCloudColorHandlerGenericField<PointNormalT> tgt_color_handler (cloud_target, "curvature");
//  if (!tgt_color_handler.isCapable ())
//      PCL_WARN ("Cannot create curvature color handler!");

//  PointCloudColorHandlerGenericField<PointNormalT> src_color_handler (cloud_source, "curvature");
//  if (!src_color_handler.isCapable ())
//      PCL_WARN ("Cannot create curvature color handler!");


//  p->addPointCloud (cloud_target, tgt_color_handler, "target", vp_2);
//  p->addPointCloud (cloud_source, src_color_handler, "source", vp_2);

//  p->spinOnce();
//}

//////////////////////////////////////////////////////////////////////////////////
///** \brief Load a set of PCD files that we want to register together
//  * \param argc the number of arguments (pass from main ())
//  * \param argv the actual command line arguments (pass from main ())
//  * \param models the resultant vector of point cloud datasets
//  */
//void loadData (int argc, char **argv, std::vector<PCD, Eigen::aligned_allocator<PCD> > &models)
//{
//  std::string extension (".pcd");
//  // Suppose the first argument is the actual test model
//  for (int i = 1; i < argc; i++)
//  {
//    std::string fname = std::string (argv[i]);
//    // Needs to be at least 5: .plot
//    if (fname.size () <= extension.size ())
//      continue;

//    std::transform (fname.begin (), fname.end (), fname.begin (), (int(*)(int))tolower);

//    //check that the argument is a pcd file
//    if (fname.compare (fname.size () - extension.size (), extension.size (), extension) == 0)
//    {
//      // Load the cloud and saves it into the global list of models
//      PCD m;
//      m.f_name = argv[i];
//      pcl::io::loadPCDFile (argv[i], *m.cloud);
//      //remove NAN points from the cloud
//      std::vector<int> indices;
//      pcl::removeNaNFromPointCloud(*m.cloud,*m.cloud, indices);

//      models.push_back (m);
//    }
//  }
//}


//////////////////////////////////////////////////////////////////////////////////
///** \brief Align a pair of PointCloud datasets and return the result
//  * \param cloud_src the source PointCloud
//  * \param cloud_tgt the target PointCloud
//  * \param output the resultant aligned source PointCloud
//  * \param final_transform the resultant transform between source and target
//  */
//void pairAlign (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample = false)
//{
//  //
//  // Downsample for consistency and speed
//  // \note enable this for large datasets
//  PointCloud::Ptr src (new PointCloud);
//  PointCloud::Ptr tgt (new PointCloud);
//  pcl::VoxelGrid<PointT> grid;
//  if (downsample)
//  {
//    grid.setLeafSize (0.05, 0.05, 0.05);
//    grid.setInputCloud (cloud_src);
//    grid.filter (*src);

//    grid.setInputCloud (cloud_tgt);
//    grid.filter (*tgt);
//  }
//  else
//  {
//    src = cloud_src;
//    tgt = cloud_tgt;
//  }


//  // Compute surface normals and curvature
//  PointCloudWithNormals::Ptr points_with_normals_src (new PointCloudWithNormals);
//  PointCloudWithNormals::Ptr points_with_normals_tgt (new PointCloudWithNormals);

//  pcl::NormalEstimation<PointT, PointNormalT> norm_est;
//  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
//  norm_est.setSearchMethod (tree);
//  norm_est.setKSearch (30);

//  norm_est.setInputCloud (src);
//  norm_est.compute (*points_with_normals_src);
//  pcl::copyPointCloud (*src, *points_with_normals_src);

//  norm_est.setInputCloud (tgt);
//  norm_est.compute (*points_with_normals_tgt);
//  pcl::copyPointCloud (*tgt, *points_with_normals_tgt);

//  //
//  // Instantiate our custom point representation (defined above) ...
//  MyPointRepresentation point_representation;
//  // ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
//  float alpha[4] = {1.0, 1.0, 1.0, 1.0};
//  point_representation.setRescaleValues (alpha);

//  //
//  // Align
//  pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
//  reg.setTransformationEpsilon (1e-6);
//  // Set the maximum distance between two correspondences (src<->tgt) to 10cm
//  // Note: adjust this based on the size of your datasets
//  reg.setMaxCorrespondenceDistance (0.1);
//  // Set the point representation
//  reg.setPointRepresentation (boost::make_shared<const MyPointRepresentation> (point_representation));

//  reg.setInputSource (points_with_normals_src);
//  reg.setInputTarget (points_with_normals_tgt);



//  //
//  // Run the same optimization in a loop and visualize the results
//  Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
//  PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
//  reg.setMaximumIterations (2);
//  for (int i = 0; i < 30; ++i)
//  {
//    PCL_INFO ("Iteration Nr. %d.\n", i);

//    // save cloud for visualization purpose
//    points_with_normals_src = reg_result;

//    // Estimate
//    reg.setInputSource (points_with_normals_src);
//    reg.align (*reg_result);

//    //accumulate transformation between each Iteration
//    Ti = reg.getFinalTransformation () * Ti;

//    //if the difference between this transformation and the previous one
//    //is smaller than the threshold, refine the process by reducing
//    //the maximal correspondence distance
//    if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
//      reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.001);

//    prev = reg.getLastIncrementalTransformation ();

//    // visualize current state
//    showCloudsRight(points_with_normals_tgt, points_with_normals_src);
//  }

//  //
//  // Get the transformation from target to source
//  targetToSource = Ti.inverse();

//  //
//  // Transform target back in source frame
//  pcl::transformPointCloud (*cloud_tgt, *output, targetToSource);

//  p->removePointCloud ("source");
//  p->removePointCloud ("target");

//  PointCloudColorHandlerCustom<PointT> cloud_tgt_h (output, 0, 255, 0);
//  PointCloudColorHandlerCustom<PointT> cloud_src_h (cloud_src, 255, 0, 0);
//  p->addPointCloud (output, cloud_tgt_h, "target", vp_2);
//  p->addPointCloud (cloud_src, cloud_src_h, "source", vp_2);

//  PCL_INFO ("Press q to continue the registration.\n");
//  p->spin ();

//  p->removePointCloud ("source");
//  p->removePointCloud ("target");

//  //add the source to the transformed target
//  *output += *cloud_src;

//  final_transform = targetToSource;
// }


///* ---[ */
//int main (int argc, char** argv)
//{
//  // Load data
//  std::vector<PCD, Eigen::aligned_allocator<PCD> > data;
//  loadData (argc, argv, data);

//  // Check user input
//  if (data.empty ())
//  {
//    PCL_ERROR ("Syntax is: %s <source.pcd> <target.pcd> [*]", argv[0]);
//    PCL_ERROR ("[*] - multiple files can be added. The registration results of (i, i+1) will be registered against (i+2), etc");
//    return (-1);
//  }
//  PCL_INFO ("Loaded %d datasets.", (int)data.size ());

//  // Create a PCLVisualizer object
//  p = new pcl::visualization::PCLVisualizer (argc, argv, "Pairwise Incremental Registration example");
//  p->createViewPort (0.0, 0, 0.5, 1.0, vp_1);
//  p->createViewPort (0.5, 0, 1.0, 1.0, vp_2);

//  PointCloud::Ptr result (new PointCloud), source, target;
//  Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity (), pairTransform;

//  for (size_t i = 1; i < data.size (); ++i)
//  {
//    source = data[i-1].cloud;
//    target = data[i].cloud;

//    // Add visualization data
//    showCloudsLeft(source, target);

//    PointCloud::Ptr temp (new PointCloud);
//    PCL_INFO ("Aligning %s (%d) with %s (%d).\n", data[i-1].f_name.c_str (), source->points.size (), data[i].f_name.c_str (), target->points.size ());
//    pairAlign (source, target, temp, pairTransform, true);

//    //transform current pair into the global transform
//    pcl::transformPointCloud (*temp, *result, GlobalTransform);

//    //update the global transform
//    GlobalTransform = GlobalTransform * pairTransform;

//    //save aligned pair, transformed into the first cloud's frame
//    std::stringstream ss;
//    ss << i << ".pcd";
//    pcl::io::savePCDFile (ss.str (), *result, true);

//  }
//}
