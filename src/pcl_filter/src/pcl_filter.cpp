/******----------------------------------voxel-downsample---------------------------------***/
//#include <iostream>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//#include <pcl/filters/voxel_grid.h>

//int
//main (int argc, char** argv)
//{
//  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
//  pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());

//  // Fill in the cloud data
//  pcl::PCDReader reader;
//  // Replace the path below with the path where you saved your file
//  reader.read ("/home/zb/BoZhan/PCL/table_scene_lms400.pcd", *cloud); // Remember to download the file first!

//  std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
//       << " data points (" << pcl::getFieldsList (*cloud) << ").";

//  // Create the filtering object
//  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
//  sor.setInputCloud (cloud);
//  sor.setLeafSize (0.01f, 0.01f, 0.01f);
//  sor.filter (*cloud_filtered);

//  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
//       << " data points (" << pcl::getFieldsList (*cloud_filtered) << ").";

//  pcl::PCDWriter writer;
//  writer.write ("table_scene_lms400_downsampled.pcd", *cloud_filtered,
//         Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);

//  return (0);
//}
/******************************************end********************************************/

/******----------------------------------RadiusOutlier removal filter---------------------------------***/
//#include <iostream>
//#include <pcl/point_types.h>
//#include <pcl/filters/radius_outlier_removal.h>
//#include <pcl/filters/conditional_removal.h>
//#include <pcl/io/pcd_io.h>

//int
// main (int argc, char** argv)
//{

//  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
//  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
//  pcl::PCLPointCloud2::Ptr cloud_filtered_save (new pcl::PCLPointCloud2 ());

//  // Fill in the cloud data
//  pcl::PCDReader reader;
//  // Replace the path below with the path where you saved your file
//  reader.read ("/home/zb/BoZhan/PCL/example_pcd/table_scene_lms400.pcd", *cloud); // Remember to download the file first!
//  std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
//       << " data points (" << pcl::getFieldsList (*cloud) << ").";

//  pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
//  // build the filter
//  outrem.setInputCloud(cloud);
//  outrem.setRadiusSearch(0.005);
//  outrem.setMinNeighborsInRadius (5);
//  std::cerr << "it is ok! " << std::endl;
//  // apply filter
//  outrem.filter (*cloud_filtered);


//  pcl::toPCLPointCloud2(*cloud_filtered,*cloud_filtered_save);

//  std::cerr << "Cloud after filtering: " << cloud_filtered->width * cloud_filtered->height<< " data points (" << pcl::getFieldsList (*cloud_filtered) << ")."<< std::endl;
//  pcl::PCDWriter writer;
//  writer.write ("table_scene_lms400_filtered.pcd", *cloud_filtered_save,Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);

//  return (0);
//}


/******----------------------------------create range images---------------------------------***/
//#include <iostream>
//#include <boost/thread/thread.hpp>
//#include <pcl/range_image/range_image.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/visualization/range_image_visualizer.h>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/features/range_image_border_extractor.h>
//#include <pcl/keypoints/narf_keypoint.h>
//#include <pcl/features/narf_descriptor.h>
//#include <pcl/console/parse.h>

//int main (int argc, char** argv) {
//  pcl::PointCloud<pcl::PointXYZ> pointCloud;

//  // Generate the data
//  if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/zb/BoZhan/PCL/example_pcd/table_scene.pcd", pointCloud) == -1) //* 读入PCD格式的文件，如果文件不存在，返回-1
//  {
//      PCL_ERROR("Couldn't read file model.pcd \n"); //文件不存在时，返回错误，终止程序。
//      return (-1);
//  }

////  for (float y=-0.5f; y<=0.5f; y+=0.01f) {
////    for (float z=-0.5f; z<=0.5f; z+=0.01f) {
////      pcl::PointXYZ point;
////      point.x = 2.0f - y;
////      point.y = y;
////      point.z = z;
////      pointCloud.points.push_back(point);
////    }
////  }
////  pointCloud.width = (uint32_t) pointCloud.points.size();
////  pointCloud.height = 1;

//  // We now want to create a range image from the above point cloud, with a 1deg angular resolution
//  float angularResolution = (float) (  0.03f * (M_PI/180.0f));  //   1.0 degree in radians
//  float maxAngleWidth     = (float) (360.0f * (M_PI/180.0f));  // 360.0 degree in radians
//  float maxAngleHeight    = (float) (180.0f * (M_PI/180.0f));  // 180.0 degree in radians
//  Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, -18.0f);
////  Eigen::Affine3f sensorPose (Eigen::Affine3f::Identity ());
//  pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
//  float noiseLevel=0.00;
//  float minRange = 0.0f;
//  int borderSize = 1;

//  boost::shared_ptr<pcl::RangeImage> range_image_ptr (new pcl::RangeImage);
//  pcl::RangeImage& rangeImage = *range_image_ptr;
//  rangeImage.createFromPointCloud(pointCloud, angularResolution, maxAngleWidth, maxAngleHeight,
//                                  sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);

//  std::cout<<"it is ok!"<<std::endl;
//  std::cout << rangeImage << "\n";

//  // --------------------------------------------
//  // -----Open 3D viewer and add point cloud-----
//  // --------------------------------------------
//  // The color we will be using
//  float bckgr_gray_level = 0.0;  // Black
//  float txt_gray_lvl = 1.0 - bckgr_gray_level;
//  pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud_temp(new pcl::PointCloud<pcl::PointXYZ>);
//  *pointCloud_temp = pointCloud;
//  pcl::visualization::PCLVisualizer viewer ("range image demo");
//  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_in_color_h(pointCloud_temp, (int) 255 * txt_gray_lvl, (int) 255 * txt_gray_lvl,
//                                                                                    (int) 255 * txt_gray_lvl);
//  viewer.addPointCloud (pointCloud_temp, cloud_in_color_h,"range image cloud");
//  viewer.addCoordinateSystem(0.1);
//  // --------------------------
//  // -----Show range image-----
//  // --------------------------
//  pcl::visualization::RangeImageVisualizer range_image_widget ("Range image");
//  range_image_widget.showRangeImage (rangeImage);
//  //--------------------
//  // -----Main loop-----
//  //--------------------
//  while (!viewer.wasStopped ())
//  {
//    range_image_widget.spinOnce ();  // process GUI events
//    viewer.spinOnce ();
//    pcl_sleep(0.01);
//  }
//}


///******----------------------------------range image border extraction---------------------------------***/
//#include <iostream>

//#include <boost/thread/thread.hpp>
//#include <pcl/range_image/range_image.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/visualization/range_image_visualizer.h>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/features/range_image_border_extractor.h>
//#include <pcl/console/parse.h>

//typedef pcl::PointXYZ PointType;

//// --------------------
//// -----Parameters-----
//// --------------------
//float angular_resolution = 0.03f;
//pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
//bool setUnseenToMaxRange = false;

//// --------------
//// -----Help-----
//// --------------
//void
//printUsage (const char* progName)
//{
//  std::cout << "\n\nUsage: "<<progName<<" [options] <scene.pcd>\n\n"
//            << "Options:\n"
//            << "-------------------------------------------\n"
//            << "-r <float>   angular resolution in degrees (default "<<angular_resolution<<")\n"
//            << "-c <int>     coordinate frame (default "<< (int)coordinate_frame<<")\n"
//            << "-m           Treat all unseen points to max range\n"
//            << "-h           this help\n"
//            << "\n\n";
//}

//// --------------
//// -----Main-----
//// --------------
//int
//main (int argc, char** argv)
//{
//  // --------------------------------------
//  // -----Parse Command Line Arguments-----
//  // --------------------------------------
//  if (pcl::console::find_argument (argc, argv, "-h") >= 0)
//  {
//    printUsage (argv[0]);
//    return 0;
//  }
//  if (pcl::console::find_argument (argc, argv, "-m") >= 0)
//  {
//    setUnseenToMaxRange = true;
//    cout << "Setting unseen values in range image to maximum range readings.\n";
//  }
//  int tmp_coordinate_frame;
//  if (pcl::console::parse (argc, argv, "-c", tmp_coordinate_frame) >= 0)
//  {
//    coordinate_frame = pcl::RangeImage::CoordinateFrame (tmp_coordinate_frame);
//    cout << "Using coordinate frame "<< (int)coordinate_frame<<".\n";
//  }
//  if (pcl::console::parse (argc, argv, "-r", angular_resolution) >= 0)
//    cout << "Setting angular resolution to "<<angular_resolution<<"deg.\n";
//  angular_resolution = pcl::deg2rad (angular_resolution);

//  // ------------------------------------------------------------------
//  // -----Read pcd file or create example point cloud if not given-----
//  // ------------------------------------------------------------------
//  pcl::PointCloud<PointType>::Ptr point_cloud_ptr (new pcl::PointCloud<PointType>);
//  pcl::PointCloud<PointType>& point_cloud = *point_cloud_ptr;
//  pcl::PointCloud<pcl::PointWithViewpoint> far_ranges;
//  Eigen::Affine3f scene_sensor_pose (Eigen::Affine3f::Identity ());
//  std::vector<int> pcd_filename_indices = pcl::console::parse_file_extension_argument (argc, argv, "pcd");
//  if (!pcd_filename_indices.empty ())
//  {
//    std::string filename = argv[pcd_filename_indices[0]];
//    if (pcl::io::loadPCDFile (filename, point_cloud) == -1)
//    {
//      cout << "Was not able to open file \""<<filename<<"\".\n";
//      printUsage (argv[0]);
//      return 0;
//    }
////    scene_sensor_pose = Eigen::Affine3f (Eigen::Translation3f (point_cloud.sensor_origin_[0],
////                                                               point_cloud.sensor_origin_[1],
////                                                               point_cloud.sensor_origin_[2])) *
////                        Eigen::Affine3f (point_cloud.sensor_orientation_);
//    scene_sensor_pose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);

//    std::string far_ranges_filename = pcl::getFilenameWithoutExtension (filename)+"_far_ranges.pcd";
//    if (pcl::io::loadPCDFile(far_ranges_filename.c_str(), far_ranges) == -1)
//      std::cout << "Far ranges file \""<<far_ranges_filename<<"\" does not exists.\n";
//  }
//  else
//  {
//    cout << "\nNo *.pcd file given => Genarating example point cloud.\n\n";
//    for (float x=-0.5f; x<=0.5f; x+=0.01f)
//    {
//      for (float y=-0.5f; y<=0.5f; y+=0.01f)
//      {
//        PointType point;  point.x = x;  point.y = y;  point.z = 2.0f - y;
//        point_cloud.points.push_back (point);
//      }
//    }
//    point_cloud.width = (int) point_cloud.points.size ();  point_cloud.height = 1;
//  }

//  // -----------------------------------------------
//  // -----Create RangeImage from the PointCloud-----
//  // -----------------------------------------------
//  float noise_level = 0.0;
//  float min_range = 0.0f;
//  int border_size = 1;
//  boost::shared_ptr<pcl::RangeImage> range_image_ptr (new pcl::RangeImage);
//  pcl::RangeImage& range_image = *range_image_ptr;
//  range_image.createFromPointCloud (point_cloud, angular_resolution, pcl::deg2rad (360.0f), pcl::deg2rad (180.0f),
//                                   scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);
//  range_image.integrateFarRanges (far_ranges);
//  if (setUnseenToMaxRange)
//    range_image.setUnseenToMaxRange ();

//  // --------------------------------------------
//  // -----Open 3D viewer and add point cloud-----
//  // --------------------------------------------
//  pcl::visualization::PCLVisualizer viewer ("3D Viewer");
//  viewer.setBackgroundColor (1, 1, 1);
//  viewer.addCoordinateSystem (0.1f);
//  pcl::visualization::PointCloudColorHandlerCustom<PointType> point_cloud_color_handler (point_cloud_ptr, 0, 0, 0);
//  viewer.addPointCloud (point_cloud_ptr, point_cloud_color_handler, "original point cloud");
//  //PointCloudColorHandlerCustom<pcl::PointWithRange> range_image_color_handler (range_image_ptr, 150, 150, 150);
//  //viewer.addPointCloud (range_image_ptr, range_image_color_handler, "range image");
//  //viewer.setPointCloudRenderingProperties (PCL_VISUALIZER_POINT_SIZE, 2, "range image");

//  // -------------------------
//  // -----Extract borders-----
//  // -------------------------
//  pcl::RangeImageBorderExtractor border_extractor (&range_image);
//  pcl::PointCloud<pcl::BorderDescription> border_descriptions;
//  border_extractor.compute (border_descriptions);

//  // ----------------------------------
//  // -----Show points in 3D viewer-----
//  // ----------------------------------
//  pcl::PointCloud<pcl::PointWithRange>::Ptr border_points_ptr(new pcl::PointCloud<pcl::PointWithRange>),
//                                            veil_points_ptr(new pcl::PointCloud<pcl::PointWithRange>),
//                                            shadow_points_ptr(new pcl::PointCloud<pcl::PointWithRange>);
//  pcl::PointCloud<pcl::PointWithRange>& border_points = *border_points_ptr,
//                                      & veil_points = * veil_points_ptr,
//                                      & shadow_points = *shadow_points_ptr;
//  for (int y=0; y< (int)range_image.height; ++y)
//  {
//    for (int x=0; x< (int)range_image.width; ++x)
//    {
//      if (border_descriptions.points[y*range_image.width + x].traits[pcl::BORDER_TRAIT__OBSTACLE_BORDER])
//        border_points.points.push_back (range_image.points[y*range_image.width + x]);
//      if (border_descriptions.points[y*range_image.width + x].traits[pcl::BORDER_TRAIT__VEIL_POINT])
//        veil_points.points.push_back (range_image.points[y*range_image.width + x]);
//      if (border_descriptions.points[y*range_image.width + x].traits[pcl::BORDER_TRAIT__SHADOW_BORDER])
//        shadow_points.points.push_back (range_image.points[y*range_image.width + x]);
//    }
//  }
//  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> border_points_color_handler (border_points_ptr, 0, 255, 0);
//  viewer.addPointCloud<pcl::PointWithRange> (border_points_ptr, border_points_color_handler, "border points");
//  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "border points");
//  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> veil_points_color_handler (veil_points_ptr, 255, 0, 0);
//  viewer.addPointCloud<pcl::PointWithRange> (veil_points_ptr, veil_points_color_handler, "veil points");
//  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "veil points");
//  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> shadow_points_color_handler (shadow_points_ptr, 0, 255, 255);
//  viewer.addPointCloud<pcl::PointWithRange> (shadow_points_ptr, shadow_points_color_handler, "shadow points");
//  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "shadow points");

//  //-------------------------------------
//  // -----Show points on range image-----
//  // ------------------------------------
//  pcl::visualization::RangeImageVisualizer* range_image_borders_widget = NULL;
//  range_image_borders_widget =
//    pcl::visualization::RangeImageVisualizer::getRangeImageBordersWidget (range_image, -std::numeric_limits<float>::infinity (), std::numeric_limits<float>::infinity (), false,
//                                                                          border_descriptions, "Range image with borders");
//  // -------------------------------------


//  //--------------------
//  // -----Main loop-----
//  //--------------------
//  while (!viewer.wasStopped ())
//  {
//    range_image_borders_widget->spinOnce ();
//    viewer.spinOnce ();
//    pcl_sleep(0.01);
//  }
//}

///******----------------------------------alignment_prerejective---------------------------------***/
//#include <Eigen/Core>
//#include <pcl/point_types.h>
//#include <pcl/point_cloud.h>
//#include <pcl/common/time.h>
//#include <pcl/console/print.h>
//#include <pcl/features/normal_3d_omp.h>
//#include <pcl/features/fpfh_omp.h>
//#include <pcl/filters/filter.h>
//#include <pcl/filters/voxel_grid.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/registration/icp.h>
//#include <pcl/registration/sample_consensus_prerejective.h>
//#include <pcl/segmentation/sac_segmentation.h>
//#include <pcl/visualization/pcl_visualizer.h>

//// Types
//typedef pcl::PointNormal PointNT;
//typedef pcl::PointCloud<PointNT> PointCloudT;
//typedef pcl::FPFHSignature33 FeatureT;
//typedef pcl::FPFHEstimationOMP<PointNT,PointNT,FeatureT> FeatureEstimationT;
//typedef pcl::PointCloud<FeatureT> FeatureCloudT;
//typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerT;

//// Align a rigid object to a scene with clutter and occlusions
//int
//main (int argc, char **argv)
//{
//  // Point clouds
//  PointCloudT::Ptr object (new PointCloudT);
//  PointCloudT::Ptr object_aligned (new PointCloudT);
//  PointCloudT::Ptr scene (new PointCloudT);
//  FeatureCloudT::Ptr object_features (new FeatureCloudT);
//  FeatureCloudT::Ptr scene_features (new FeatureCloudT);

//  // Get input object and scene
//  if (argc != 3)
//  {
//    pcl::console::print_error ("Syntax is: %s object.pcd scene.pcd\n", argv[0]);
//    return (1);
//  }

//  // Load object and scene
//  pcl::console::print_highlight ("Loading point clouds...\n");
//  if (pcl::io::loadPCDFile<PointNT> (argv[1], *object) < 0 ||
//      pcl::io::loadPCDFile<PointNT> (argv[2], *scene) < 0)
//  {
//    pcl::console::print_error ("Error loading object/scene file!\n");
//    return (1);
//  }


//  // Downsample
//  pcl::console::print_highlight ("Downsampling...\n");
//  pcl::VoxelGrid<PointNT> grid;
//  const float leaf = 0.005f;
//  grid.setLeafSize (leaf, leaf, leaf);
//  grid.setInputCloud (object);
//  grid.filter (*object);
//  grid.setInputCloud (scene);
//  grid.filter (*scene);


//  // Estimate normals for scene
//  pcl::console::print_highlight ("Estimating scene normals...\n");
//  pcl::NormalEstimationOMP<PointNT,PointNT> nest;
//  nest.setRadiusSearch (0.01);
//  nest.setInputCloud (scene);
//  nest.compute (*scene);
//  nest.setInputCloud (object);
//  nest.compute (*object);

//  std::cout << "object " << object->size()<<" points "  <<std::endl;
//  std::cout << "scene " << scene->size()<<" points "  <<std::endl;


//  // Estimate features
//  pcl::console::print_highlight ("Estimating features...\n");
//  FeatureEstimationT fest;
//  fest.setRadiusSearch (0.025);
//  fest.setInputCloud (object);
//  fest.setInputNormals (object);
//  fest.compute (*object_features);
//  fest.setInputCloud (scene);
//  fest.setInputNormals (scene);
//  fest.compute (*scene_features);

//  // Perform alignment
//  pcl::console::print_highlight ("Starting alignment...\n");
//  pcl::SampleConsensusPrerejective<PointNT,PointNT,FeatureT> align;
//  align.setInputSource (object);
//  align.setSourceFeatures (object_features);
//  align.setInputTarget (scene);
//  align.setTargetFeatures (scene_features);
//  align.setMaximumIterations (50000); // Number of RANSAC iterations
//  align.setNumberOfSamples (3); // Number of points to sample for generating/prerejecting a pose
//  align.setCorrespondenceRandomness (5); // Number of nearest features to use
//  align.setSimilarityThreshold (0.9f); // Polygonal edge length similarity threshold
//  align.setMaxCorrespondenceDistance (2.5f * leaf); // Inlier threshold
//  align.setInlierFraction (0.25f); // Required inlier fraction for accepting a pose hypothesis
//  {
//    pcl::ScopeTime t("Alignment");
//    align.align (*object_aligned);
//  }

//  if (align.hasConverged ())
//  {
//    // Print results
//    printf ("\n");
//    Eigen::Matrix4f transformation = align.getFinalTransformation ();
//    pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (0,0), transformation (0,1), transformation (0,2));
//    pcl::console::print_info ("R = | %6.3f %6.3f %6.3f | \n", transformation (1,0), transformation (1,1), transformation (1,2));
//    pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (2,0), transformation (2,1), transformation (2,2));
//    pcl::console::print_info ("\n");
//    pcl::console::print_info ("t = < %0.3f, %0.3f, %0.3f >\n", transformation (0,3), transformation (1,3), transformation (2,3));
//    pcl::console::print_info ("\n");
//    pcl::console::print_info ("Inliers: %i/%i\n", align.getInliers ().size (), object->size ());

//    // Show alignment
//    pcl::visualization::PCLVisualizer visu("Alignment");
//    visu.addPointCloud (scene, ColorHandlerT (scene, 0.0, 255.0, 0.0), "scene");
//    visu.addPointCloud (object_aligned, ColorHandlerT (object_aligned, 0.0, 0.0, 255.0), "object_aligned");
//    visu.spin ();
//  }
//  else
//  {
//    pcl::console::print_error ("Alignment failed!\n");
//    return (1);
//  }

//  return (0);
//}


/////******----------------------------------template_alignment---------------------------------***/
//#include <limits>
//#include <fstream>
//#include <vector>
//#include <Eigen/Core>
//#include <pcl/point_types.h>
//#include <pcl/point_cloud.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/filters/passthrough.h>
//#include <pcl/filters/voxel_grid.h>
//#include <pcl/features/normal_3d.h>
//#include <pcl/features/fpfh.h>
//#include <pcl/registration/ia_ransac.h>

//class FeatureCloud
//{
//  public:
//    // A bit of shorthand
//    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
//    typedef pcl::PointCloud<pcl::Normal> SurfaceNormals;
//    typedef pcl::PointCloud<pcl::FPFHSignature33> LocalFeatures;
//    typedef pcl::search::KdTree<pcl::PointXYZ> SearchMethod;

//    FeatureCloud () :
//      search_method_xyz_ (new SearchMethod),
//      normal_radius_ (0.02f),
//      feature_radius_ (0.02f)
//    {}

//    ~FeatureCloud () {}

//    // Process the given cloud
//    void
//    setInputCloud (PointCloud::Ptr xyz)
//    {
//      xyz_ = xyz;
//      processInput ();
//    }

//    // Load and process the cloud in the given PCD file
//    void
//    loadInputCloud (const std::string &pcd_file)
//    {
//      xyz_ = PointCloud::Ptr (new PointCloud);
//      pcl::io::loadPCDFile (pcd_file, *xyz_);
//      processInput ();
//    }

//    // Get a pointer to the cloud 3D points
//    PointCloud::Ptr
//    getPointCloud () const
//    {
//      return (xyz_);
//    }

//    // Get a pointer to the cloud of 3D surface normals
//    SurfaceNormals::Ptr
//    getSurfaceNormals () const
//    {
//      return (normals_);
//    }

//    // Get a pointer to the cloud of feature descriptors
//    LocalFeatures::Ptr
//    getLocalFeatures () const
//    {
//      return (features_);
//    }

//  protected:
//    // Compute the surface normals and local features
//    void
//    processInput ()
//    {
//      computeSurfaceNormals ();
//      computeLocalFeatures ();
//    }

//    // Compute the surface normals
//    void
//    computeSurfaceNormals ()
//    {
//      normals_ = SurfaceNormals::Ptr (new SurfaceNormals);

//      pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> norm_est;
//      norm_est.setInputCloud (xyz_);
//      norm_est.setSearchMethod (search_method_xyz_);
//      norm_est.setRadiusSearch (normal_radius_);
//      norm_est.compute (*normals_);
//    }

//    // Compute the local feature descriptors
//    void
//    computeLocalFeatures ()
//    {
//      features_ = LocalFeatures::Ptr (new LocalFeatures);

//      pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
//      fpfh_est.setInputCloud (xyz_);
//      fpfh_est.setInputNormals (normals_);
//      fpfh_est.setSearchMethod (search_method_xyz_);
//      fpfh_est.setRadiusSearch (feature_radius_);
//      fpfh_est.compute (*features_);
//    }

//  private:
//    // Point cloud data
//    PointCloud::Ptr xyz_;
//    SurfaceNormals::Ptr normals_;
//    LocalFeatures::Ptr features_;
//    SearchMethod::Ptr search_method_xyz_;

//    // Parameters
//    float normal_radius_;
//    float feature_radius_;
//};

//class TemplateAlignment
//{
//  public:

//    // A struct for storing alignment results
//    struct Result
//    {
//      float fitness_score;
//      Eigen::Matrix4f final_transformation;
//      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//    };

//    TemplateAlignment () :
//      min_sample_distance_ (0.05f),
//      max_correspondence_distance_ (0.01f*0.01f),
//      nr_iterations_ (500)
//    {
//      // Intialize the parameters in the Sample Consensus Intial Alignment (SAC-IA) algorithm
//      sac_ia_.setMinSampleDistance (min_sample_distance_);
//      sac_ia_.setMaxCorrespondenceDistance (max_correspondence_distance_);
//      sac_ia_.setMaximumIterations (nr_iterations_);
//    }

//    ~TemplateAlignment () {}

//    // Set the given cloud as the target to which the templates will be aligned
//    void
//    setTargetCloud (FeatureCloud &target_cloud)
//    {
//      target_ = target_cloud;
//      sac_ia_.setInputTarget (target_cloud.getPointCloud ());
//      sac_ia_.setTargetFeatures (target_cloud.getLocalFeatures ());
//    }

//    // Add the given cloud to the list of template clouds
//    void
//    addTemplateCloud (FeatureCloud &template_cloud)
//    {
//      templates_.push_back (template_cloud);
//    }

//    // Align the given template cloud to the target specified by setTargetCloud ()
//    void
//    align (FeatureCloud &template_cloud, TemplateAlignment::Result &result)
//    {
//      sac_ia_.setInputCloud (template_cloud.getPointCloud ());
//      sac_ia_.setSourceFeatures (template_cloud.getLocalFeatures ());

//      pcl::PointCloud<pcl::PointXYZ> registration_output;
//      sac_ia_.align (registration_output);

//      result.fitness_score = (float) sac_ia_.getFitnessScore (max_correspondence_distance_);
//      result.final_transformation = sac_ia_.getFinalTransformation ();
//    }

//    // Align all of template clouds set by addTemplateCloud to the target specified by setTargetCloud ()
//    void
//    alignAll (std::vector<TemplateAlignment::Result, Eigen::aligned_allocator<Result> > &results)
//    {
//      results.resize (templates_.size ());
//      for (size_t i = 0; i < templates_.size (); ++i)
//      {
//        align (templates_[i], results[i]);
//      }
//    }

//    // Align all of template clouds to the target cloud to find the one with best alignment score
//    int
//    findBestAlignment (TemplateAlignment::Result &result)
//    {
//      // Align all of the templates to the target cloud
//      std::vector<Result, Eigen::aligned_allocator<Result> > results;
//      alignAll (results);

//      // Find the template with the best (lowest) fitness score
//      float lowest_score = std::numeric_limits<float>::infinity ();
//      int best_template = 0;
//      for (size_t i = 0; i < results.size (); ++i)
//      {
//        const Result &r = results[i];
//        if (r.fitness_score < lowest_score)
//        {
//          lowest_score = r.fitness_score;
//          best_template = (int) i;
//        }
//      }

//      // Output the best alignment
//      result = results[best_template];
//      return (best_template);
//    }

//  private:
//    // A list of template clouds and the target to which they will be aligned
//    std::vector<FeatureCloud> templates_;
//    FeatureCloud target_;

//    // The Sample Consensus Initial Alignment (SAC-IA) registration routine and its parameters
//    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia_;
//    float min_sample_distance_;
//    float max_correspondence_distance_;
//    int nr_iterations_;
//};

//// Align a collection of object templates to a sample point cloud
//int
//main (int argc, char **argv)
//{
//  if (argc < 3)
//  {
//    printf ("No target PCD file given!\n");
//    return (-1);
//  }

//  // Load the object templates specified in the object_templates.txt file
//  std::vector<FeatureCloud> object_templates;
//  std::ifstream input_stream (argv[1]);
//  object_templates.resize (0);
//  std::string pcd_filename;
//  while (input_stream.good ())
//  {
//    std::getline (input_stream, pcd_filename);
//    if (pcd_filename.empty () || pcd_filename.at (0) == '#') // Skip blank lines or comments
//      continue;

//    FeatureCloud template_cloud;
//    template_cloud.loadInputCloud (pcd_filename);
//    object_templates.push_back (template_cloud);
//  }
//  input_stream.close ();

//  // Load the target cloud PCD file
//  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
//  pcl::io::loadPCDFile (argv[2], *cloud);

//  // Preprocess the cloud by...
//  // ...removing distant points
//  const float depth_limit = 1.0;
//  pcl::PassThrough<pcl::PointXYZ> pass;
//  pass.setInputCloud (cloud);
//  pass.setFilterFieldName ("z");
//  pass.setFilterLimits (0, depth_limit);
//  pass.filter (*cloud);

//  // ... and downsampling the point cloud
//  const float voxel_grid_size = 0.005f;
//  pcl::VoxelGrid<pcl::PointXYZ> vox_grid;
//  vox_grid.setInputCloud (cloud);
//  vox_grid.setLeafSize (voxel_grid_size, voxel_grid_size, voxel_grid_size);
//  //vox_grid.filter (*cloud); // Please see this http://www.pcl-developers.org/Possible-problem-in-new-VoxelGrid-implementation-from-PCL-1-5-0-td5490361.html
//  pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud (new pcl::PointCloud<pcl::PointXYZ>);
//  vox_grid.filter (*tempCloud);
//  cloud = tempCloud;

//  // Assign to the target FeatureCloud
//  FeatureCloud target_cloud;
//  target_cloud.setInputCloud (cloud);

//  // Set the TemplateAlignment inputs
//  TemplateAlignment template_align;
//  for (size_t i = 0; i < object_templates.size (); ++i)
//  {
//    template_align.addTemplateCloud (object_templates[i]);
//  }
//  template_align.setTargetCloud (target_cloud);

//  // Find the best template alignment
//  TemplateAlignment::Result best_alignment;
//  int best_index = template_align.findBestAlignment (best_alignment);
//  const FeatureCloud &best_template = object_templates[best_index];

//  // Print the alignment fitness score (values less than 0.00002 are good)
//  printf ("Best fitness score: %f\n", best_alignment.fitness_score);

//  // Print the rotation matrix and translation vector
//  Eigen::Matrix3f rotation = best_alignment.final_transformation.block<3,3>(0, 0);
//  Eigen::Vector3f translation = best_alignment.final_transformation.block<3,1>(0, 3);

//  printf ("\n");
//  printf ("    | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
//  printf ("R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
//  printf ("    | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
//  printf ("\n");
//  printf ("t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));

//  // Save the aligned template for visualization
//  pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
//  pcl::transformPointCloud (*best_template.getPointCloud (), transformed_cloud, best_alignment.final_transformation);
//  pcl::io::savePCDFileBinary ("output.pcd", transformed_cloud);

//  return (0);
//}

///////******----------------------------------correspondance_grouping---------------------------------***/
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_cloud.h>
//#include <pcl/correspondence.h>
//#include <pcl/features/normal_3d_omp.h>
//#include <pcl/features/shot_omp.h>
//#include <pcl/features/board.h>
////#include <pcl/filters/uniform_sampling.h>
//#include <pcl/recognition/cg/hough_3d.h>
//#include <pcl/recognition/cg/geometric_consistency.h>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/kdtree/impl/kdtree_flann.hpp>
//#include <pcl/common/transforms.h>
//#include <pcl/console/parse.h>
//#include <pcl/filters/voxel_grid.h>
//#include <pcl/filters/filter.h>

//typedef pcl::PointXYZRGBA PointType;
//typedef pcl::Normal NormalType;
//typedef pcl::ReferenceFrame RFType;
//typedef pcl::SHOT352 DescriptorType;

//std::string model_filename_;
//std::string scene_filename_;

////Algorithm params
//bool show_keypoints_ (false);
//bool show_correspondences_ (false);
//bool use_cloud_resolution_ (false);
//bool use_hough_ (true);
//float model_ss_ (0.01f);
//float scene_ss_ (0.03f);
//float rf_rad_ (0.015f);
//float descr_rad_ (0.02f);
//float cg_size_ (0.01f);
//float cg_thresh_ (5.0f);

//void
//showHelp (char *filename)
//{
//  std::cout << std::endl;
//  std::cout << "***************************************************************************" << std::endl;
//  std::cout << "*                                                                         *" << std::endl;
//  std::cout << "*             Correspondence Grouping Tutorial - Usage Guide              *" << std::endl;
//  std::cout << "*                                                                         *" << std::endl;
//  std::cout << "***************************************************************************" << std::endl << std::endl;
//  std::cout << "Usage: " << filename << " model_filename.pcd scene_filename.pcd [Options]" << std::endl << std::endl;
//  std::cout << "Options:" << std::endl;
//  std::cout << "     -h:                     Show this help." << std::endl;
//  std::cout << "     -k:                     Show used keypoints." << std::endl;
//  std::cout << "     -c:                     Show used correspondences." << std::endl;
//  std::cout << "     -r:                     Compute the model cloud resolution and multiply" << std::endl;
//  std::cout << "                             each radius given by that value." << std::endl;
//  std::cout << "     --algorithm (Hough|GC): Clustering algorithm used (default Hough)." << std::endl;
//  std::cout << "     --model_ss val:         Model uniform sampling radius (default 0.01)" << std::endl;
//  std::cout << "     --scene_ss val:         Scene uniform sampling radius (default 0.03)" << std::endl;
//  std::cout << "     --rf_rad val:           Reference frame radius (default 0.015)" << std::endl;
//  std::cout << "     --descr_rad val:        Descriptor radius (default 0.02)" << std::endl;
//  std::cout << "     --cg_size val:          Cluster size (default 0.01)" << std::endl;
//  std::cout << "     --cg_thresh val:        Clustering threshold (default 5)" << std::endl << std::endl;
//}

//void
//parseCommandLine (int argc, char *argv[])
//{
//  //Show help
//  if (pcl::console::find_switch (argc, argv, "-h"))
//  {
//    showHelp (argv[0]);
//    exit (0);
//  }

//  //Model & scene filenames
//  std::vector<int> filenames;
//  filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
//  if (filenames.size () != 2)
//  {
//    std::cout << "Filenames missing.\n";
//    showHelp (argv[0]);
//    exit (-1);
//  }

//  model_filename_ = argv[filenames[0]];
//  scene_filename_ = argv[filenames[1]];

//  //Program behavior
//  if (pcl::console::find_switch (argc, argv, "-k"))
//  {
//    show_keypoints_ = true;
//  }
//  if (pcl::console::find_switch (argc, argv, "-c"))
//  {
//    show_correspondences_ = true;
//  }
//  if (pcl::console::find_switch (argc, argv, "-r"))
//  {
//    use_cloud_resolution_ = true;
//  }

//  std::string used_algorithm;
//  if (pcl::console::parse_argument (argc, argv, "--algorithm", used_algorithm) != -1)
//  {
//    if (used_algorithm.compare ("Hough") == 0)
//    {
//      use_hough_ = true;
//    }else if (used_algorithm.compare ("GC") == 0)
//    {
//      use_hough_ = false;
//    }
//    else
//    {
//      std::cout << "Wrong algorithm name.\n";
//      showHelp (argv[0]);
//      exit (-1);
//    }
//  }

//  //General parameters
//  pcl::console::parse_argument (argc, argv, "--model_ss", model_ss_);
//  pcl::console::parse_argument (argc, argv, "--scene_ss", scene_ss_);
//  pcl::console::parse_argument (argc, argv, "--rf_rad", rf_rad_);
//  pcl::console::parse_argument (argc, argv, "--descr_rad", descr_rad_);
//  pcl::console::parse_argument (argc, argv, "--cg_size", cg_size_);
//  pcl::console::parse_argument (argc, argv, "--cg_thresh", cg_thresh_);
//}

//double
//computeCloudResolution (const pcl::PointCloud<PointType>::ConstPtr &cloud)
//{
//  double res = 0.0;
//  int n_points = 0;
//  int nres;
//  std::vector<int> indices (2);
//  std::vector<float> sqr_distances (2);
//  pcl::search::KdTree<PointType> tree;
//  tree.setInputCloud (cloud);

//  for (size_t i = 0; i < cloud->size (); ++i)
//  {
//    if (! pcl_isfinite ((*cloud)[i].x))
//    {
//      continue;
//    }
//    //Considering the second neighbor since the first is the point itself.
//    nres = tree.nearestKSearch (i, 2, indices, sqr_distances);
//    if (nres == 2)
//    {
//      res += sqrt (sqr_distances[1]);
//      ++n_points;
//    }
//  }
//  if (n_points != 0)
//  {
//    res /= n_points;
//  }
//  return res;
//}

//int
//main (int argc, char *argv[])
//{
//  parseCommandLine (argc, argv);

//  pcl::PointCloud<PointType>::Ptr model (new pcl::PointCloud<PointType> ());
//  pcl::PointCloud<PointType>::Ptr model_keypoints (new pcl::PointCloud<PointType> ());
//  pcl::PointCloud<PointType>::Ptr scene (new pcl::PointCloud<PointType> ());
//  pcl::PointCloud<PointType>::Ptr scene_keypoints (new pcl::PointCloud<PointType> ());
//  pcl::PointCloud<NormalType>::Ptr model_normals (new pcl::PointCloud<NormalType> ());
//  pcl::PointCloud<NormalType>::Ptr scene_normals (new pcl::PointCloud<NormalType> ());
//  pcl::PointCloud<DescriptorType>::Ptr model_descriptors (new pcl::PointCloud<DescriptorType> ());
//  pcl::PointCloud<DescriptorType>::Ptr scene_descriptors (new pcl::PointCloud<DescriptorType> ());

//  //
//  //  Load clouds
//  //
//  if (pcl::io::loadPCDFile (model_filename_, *model) < 0)
//  {
//    std::cout << "Error loading model cloud." << std::endl;
//    showHelp (argv[0]);
//    return (-1);
//  }
//  if (pcl::io::loadPCDFile (scene_filename_, *scene) < 0)
//  {
//    std::cout << "Error loading scene cloud." << std::endl;
//    showHelp (argv[0]);
//    return (-1);
//  }

//  //
//  //  Set up resolution invariance
//  //
//  if (use_cloud_resolution_)
//  {
//    float resolution = static_cast<float> (computeCloudResolution (model));
//    if (resolution != 0.0f)
//    {
//      model_ss_   *= resolution;
//      scene_ss_   *= resolution;
//      rf_rad_     *= resolution;
//      descr_rad_  *= resolution;
//      cg_size_    *= resolution;
//    }

//    std::cout << "Model resolution:       " << resolution << std::endl;
//    std::cout << "Model sampling size:    " << model_ss_ << std::endl;
//    std::cout << "Scene sampling size:    " << scene_ss_ << std::endl;
//    std::cout << "LRF support radius:     " << rf_rad_ << std::endl;
//    std::cout << "SHOT descriptor radius: " << descr_rad_ << std::endl;
//    std::cout << "Clustering bin size:    " << cg_size_ << std::endl << std::endl;
//  }

//  //
//  //  Compute Normals
//  //
//  pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
//  norm_est.setKSearch (10);
//  norm_est.setInputCloud (model);
//  norm_est.compute (*model_normals);

//  norm_est.setInputCloud (scene);
//  norm_est.compute (*scene_normals);

//  //
//  //  Downsample Clouds to Extract keypoints
//  //

//  //create the filtering object
////  pcl::PCLPointCloud2::Ptr cloud_in_temp (new pcl::PCLPointCloud2 ());
////  pcl::toPCLPointCloud2(*cloud_in_un,*cloud_in_temp);
////  pcl::PCLPointCloud2::Ptr cloud_in_filtered (new pcl::PCLPointCloud2 ());


//  pcl::VoxelGrid<PointType> uniform_sampling;
//  uniform_sampling.setInputCloud (model);
//  uniform_sampling.setLeafSize (model_ss_,model_ss_,model_ss_);
//  uniform_sampling.filter (*model_keypoints);
//  std::cout << "Model total points: " << model->size () << "; Selected Keypoints: " << model_keypoints->size () << std::endl;

//  uniform_sampling.setInputCloud (scene);
//  uniform_sampling.setLeafSize (scene_ss_,scene_ss_,scene_ss_);
//  uniform_sampling.filter (*scene_keypoints);
//  std::cout << "Scene total points: " << scene->size () << "; Selected Keypoints: " << scene_keypoints->size () << std::endl;


//  //
//  //  Compute Descriptor for keypoints
//  //
//  pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est;
//  descr_est.setRadiusSearch (descr_rad_);

//  descr_est.setInputCloud (model_keypoints);
//  descr_est.setInputNormals (model_normals);
//  descr_est.setSearchSurface (model);
//  descr_est.compute (*model_descriptors);

//  descr_est.setInputCloud (scene_keypoints);
//  descr_est.setInputNormals (scene_normals);
//  descr_est.setSearchSurface (scene);
//  descr_est.compute (*scene_descriptors);

//  //
//  //  Find Model-Scene Correspondences with KdTree
//  //
//  pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());

//  pcl::KdTreeFLANN<DescriptorType> match_search;
//  match_search.setInputCloud (model_descriptors);

//  //  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
//  for (size_t i = 0; i < scene_descriptors->size (); ++i)
//  {
//    std::vector<int> neigh_indices (1);
//    std::vector<float> neigh_sqr_dists (1);
//    if (!pcl_isfinite (scene_descriptors->at (i).descriptor[0])) //skipping NaNs
//    {
//      continue;
//    }
//    int found_neighs = match_search.nearestKSearch (scene_descriptors->at (i), 1, neigh_indices, neigh_sqr_dists);
//    if(found_neighs == 1 && neigh_sqr_dists[0] < 0.25f) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
//    {
//      pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
//      model_scene_corrs->push_back (corr);
//    }
//  }
//  std::cout << "Correspondences found: " << model_scene_corrs->size () << std::endl;

//  //
//  //  Actual Clustering
//  //
//  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
//  std::vector<pcl::Correspondences> clustered_corrs;

//  //  Using Hough3D
//  if (use_hough_)
//  {
//    //
//    //  Compute (Keypoints) Reference Frames only for Hough
//    //
//    pcl::PointCloud<RFType>::Ptr model_rf (new pcl::PointCloud<RFType> ());
//    pcl::PointCloud<RFType>::Ptr scene_rf (new pcl::PointCloud<RFType> ());

//    pcl::BOARDLocalReferenceFrameEstimation<PointType, NormalType, RFType> rf_est;
//    rf_est.setFindHoles (true);
//    rf_est.setRadiusSearch (rf_rad_);

//    rf_est.setInputCloud (model_keypoints);
//    rf_est.setInputNormals (model_normals);
//    rf_est.setSearchSurface (model);
//    rf_est.compute (*model_rf);

//    rf_est.setInputCloud (scene_keypoints);
//    rf_est.setInputNormals (scene_normals);
//    rf_est.setSearchSurface (scene);
//    rf_est.compute (*scene_rf);

//    //  Clustering
//    pcl::Hough3DGrouping<PointType, PointType, RFType, RFType> clusterer;
//    clusterer.setHoughBinSize (cg_size_);
//    clusterer.setHoughThreshold (cg_thresh_);
//    clusterer.setUseInterpolation (true);
//    clusterer.setUseDistanceWeight (false);

//    clusterer.setInputCloud (model_keypoints);
//    clusterer.setInputRf (model_rf);
//    clusterer.setSceneCloud (scene_keypoints);
//    clusterer.setSceneRf (scene_rf);
//    clusterer.setModelSceneCorrespondences (model_scene_corrs);

//    //clusterer.cluster (clustered_corrs);
//    clusterer.recognize (rototranslations, clustered_corrs);
//  }
//  else // Using GeometricConsistency
//  {
//    pcl::GeometricConsistencyGrouping<PointType, PointType> gc_clusterer;
//    gc_clusterer.setGCSize (cg_size_);
//    gc_clusterer.setGCThreshold (cg_thresh_);

//    gc_clusterer.setInputCloud (model_keypoints);
//    gc_clusterer.setSceneCloud (scene_keypoints);
//    gc_clusterer.setModelSceneCorrespondences (model_scene_corrs);

//    //gc_clusterer.cluster (clustered_corrs);
//    gc_clusterer.recognize (rototranslations, clustered_corrs);
//  }

//  //
//  //  Output results
//  //
//  std::cout << "Model instances found: " << rototranslations.size () << std::endl;
//  for (size_t i = 0; i < rototranslations.size (); ++i)
//  {
//    std::cout << "\n    Instance " << i + 1 << ":" << std::endl;
//    std::cout << "        Correspondences belonging to this instance: " << clustered_corrs[i].size () << std::endl;

//    // Print the rotation matrix and translation vector
//    Eigen::Matrix3f rotation = rototranslations[i].block<3,3>(0, 0);
//    Eigen::Vector3f translation = rototranslations[i].block<3,1>(0, 3);

//    printf ("\n");
//    printf ("            | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
//    printf ("        R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
//    printf ("            | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
//    printf ("\n");
//    printf ("        t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));
//  }

//  //
//  //  Visualization
//  //
//  pcl::visualization::PCLVisualizer viewer ("Correspondence Grouping");
//  viewer.addPointCloud (scene, "scene_cloud");

//  pcl::PointCloud<PointType>::Ptr off_scene_model (new pcl::PointCloud<PointType> ());
//  pcl::PointCloud<PointType>::Ptr off_scene_model_keypoints (new pcl::PointCloud<PointType> ());

//  if (show_correspondences_ || show_keypoints_)
//  {
//    //  We are translating the model so that it doesn't end in the middle of the scene representation
//    pcl::transformPointCloud (*model, *off_scene_model, Eigen::Vector3f (-1,0,0), Eigen::Quaternionf (1, 0, 0, 0));
//    pcl::transformPointCloud (*model_keypoints, *off_scene_model_keypoints, Eigen::Vector3f (-1,0,0), Eigen::Quaternionf (1, 0, 0, 0));

//    pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_color_handler (off_scene_model, 255, 255, 128);
//    viewer.addPointCloud (off_scene_model, off_scene_model_color_handler, "off_scene_model");
//  }

//  if (show_keypoints_)
//  {
//    pcl::visualization::PointCloudColorHandlerCustom<PointType> scene_keypoints_color_handler (scene_keypoints, 0, 0, 255);
//    viewer.addPointCloud (scene_keypoints, scene_keypoints_color_handler, "scene_keypoints");
//    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scene_keypoints");

//    pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_keypoints_color_handler (off_scene_model_keypoints, 0, 0, 255);
//    viewer.addPointCloud (off_scene_model_keypoints, off_scene_model_keypoints_color_handler, "off_scene_model_keypoints");
//    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "off_scene_model_keypoints");
//  }

//  for (size_t i = 0; i < rototranslations.size (); ++i)
//  {
//    pcl::PointCloud<PointType>::Ptr rotated_model (new pcl::PointCloud<PointType> ());
//    pcl::transformPointCloud (*model, *rotated_model, rototranslations[i]);

//    std::stringstream ss_cloud;
//    ss_cloud << "instance" << i;

//    pcl::visualization::PointCloudColorHandlerCustom<PointType> rotated_model_color_handler (rotated_model, 255, 0, 0);
//    viewer.addPointCloud (rotated_model, rotated_model_color_handler, ss_cloud.str ());

//    if (show_correspondences_)
//    {
//      for (size_t j = 0; j < clustered_corrs[i].size (); ++j)
//      {
//        std::stringstream ss_line;
//        ss_line << "correspondence_line" << i << "_" << j;
//        PointType& model_point = off_scene_model_keypoints->at (clustered_corrs[i][j].index_query);
//        PointType& scene_point = scene_keypoints->at (clustered_corrs[i][j].index_match);

//        //  We are drawing a line for each pair of clustered correspondences found between the model and the scene
//        viewer.addLine<PointType, PointType> (model_point, scene_point, 0, 255, 0, ss_line.str ());
//      }
//    }
//  }

//  while (!viewer.wasStopped ())
//  {
//    viewer.spinOnce ();
//  }

//  return (0);
//}


/////////******----------------------------------global_hypothesis_verification after correspondance_grouping---------------------------------***/

///*
// * Software License Agreement (BSD License)
// *
// *  Point Cloud Library (PCL) - www.pointclouds.org
// *  Copyright (c) 2014-, Open Perception, Inc.
// *
// *  All rights reserved.
// *
// *  Redistribution and use in source and binary forms, with or without
// *  modification, are permitted provided that the following conditions
// *  are met:
// *
// *   * Redistributions of source code must retain the above copyright
// *     notice, this list of conditions and the following disclaimer.
// *   * Redistributions in binary form must reproduce the above
// *     copyright notice, this list of conditions and the following
// *     disclaimer in the documentation and/or other materials provided
// *     with the distribution.
// *   * Neither the name of the copyright holder(s) nor the names of its
// *     contributors may be used to endorse or promote products derived
// *     from this software without specific prior written permission.
// *
// *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// *  POSSIBILITY OF SUCH DAMAGE.
// *
// */

//#include <pcl/io/pcd_io.h>
//#include <pcl/point_cloud.h>
//#include <pcl/correspondence.h>
//#include <pcl/features/normal_3d_omp.h>
//#include <pcl/features/shot_omp.h>
//#include <pcl/features/board.h>
////#include <pcl/filters/uniform_sampling.h>
//#include <pcl/recognition/cg/hough_3d.h>
//#include <pcl/recognition/cg/geometric_consistency.h>
//#include <pcl/recognition/hv/hv_go.h>
//#include <pcl/registration/icp.h>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/kdtree/impl/kdtree_flann.hpp>
//#include <pcl/common/transforms.h>
//#include <pcl/console/parse.h>
//#include <pcl/filters/voxel_grid.h>
//#include <pcl/filters/filter.h>

//typedef pcl::PointXYZRGBA PointType;
//typedef pcl::Normal NormalType;
//typedef pcl::ReferenceFrame RFType;
//typedef pcl::SHOT352 DescriptorType;

//struct CloudStyle
//{
//    double r;
//    double g;
//    double b;
//    double size;

//    CloudStyle (double r,
//                double g,
//                double b,
//                double size) :
//        r (r),
//        g (g),
//        b (b),
//        size (size)
//    {
//    }
//};

//CloudStyle style_white (255.0, 255.0, 255.0, 4.0);
//CloudStyle style_red (255.0, 0.0, 0.0, 3.0);
//CloudStyle style_green (0.0, 255.0, 0.0, 5.0);
//CloudStyle style_cyan (93.0, 200.0, 217.0, 4.0);
//CloudStyle style_violet (255.0, 0.0, 255.0, 8.0);

//std::string model_filename_;
//std::string scene_filename_;

////Algorithm params
//bool show_keypoints_ (false);
//bool use_hough_ (true);
//float model_ss_ (0.02f);
//float scene_ss_ (0.02f);
//float rf_rad_ (0.015f);
//float descr_rad_ (0.02f);
//float cg_size_ (0.01f);
//float cg_thresh_ (5.0f);
//int icp_max_iter_ (5);
//float icp_corr_distance_ (0.005f);
//float hv_clutter_reg_ (5.0f);
//float hv_inlier_th_ (0.005f);
//float hv_occlusion_th_ (0.01f);
//float hv_rad_clutter_ (0.03f);
//float hv_regularizer_ (3.0f);
//float hv_rad_normals_ (0.05);
//bool hv_detect_clutter_ (true);

///**
// * Prints out Help message
// * @param filename Runnable App Name
// */
//void
//showHelp (char *filename)
//{
//  std::cout << std::endl;
//  std::cout << "***************************************************************************" << std::endl;
//  std::cout << "*                                                                         *" << std::endl;
//  std::cout << "*          Global Hypothese Verification Tutorial - Usage Guide          *" << std::endl;
//  std::cout << "*                                                                         *" << std::endl;
//  std::cout << "***************************************************************************" << std::endl << std::endl;
//  std::cout << "Usage: " << filename << " model_filename.pcd scene_filename.pcd [Options]" << std::endl << std::endl;
//  std::cout << "Options:" << std::endl;
//  std::cout << "     -h:                          Show this help." << std::endl;
//  std::cout << "     -k:                          Show keypoints." << std::endl;
//  std::cout << "     --algorithm (Hough|GC):      Clustering algorithm used (default Hough)." << std::endl;
//  std::cout << "     --model_ss val:              Model uniform sampling radius (default " << model_ss_ << ")" << std::endl;
//  std::cout << "     --scene_ss val:              Scene uniform sampling radius (default " << scene_ss_ << ")" << std::endl;
//  std::cout << "     --rf_rad val:                Reference frame radius (default " << rf_rad_ << ")" << std::endl;
//  std::cout << "     --descr_rad val:             Descriptor radius (default " << descr_rad_ << ")" << std::endl;
//  std::cout << "     --cg_size val:               Cluster size (default " << cg_size_ << ")" << std::endl;
//  std::cout << "     --cg_thresh val:             Clustering threshold (default " << cg_thresh_ << ")" << std::endl << std::endl;
//  std::cout << "     --icp_max_iter val:          ICP max iterations number (default " << icp_max_iter_ << ")" << std::endl;
//  std::cout << "     --icp_corr_distance val:     ICP correspondence distance (default " << icp_corr_distance_ << ")" << std::endl << std::endl;
//  std::cout << "     --hv_clutter_reg val:        Clutter Regularizer (default " << hv_clutter_reg_ << ")" << std::endl;
//  std::cout << "     --hv_inlier_th val:          Inlier threshold (default " << hv_inlier_th_ << ")" << std::endl;
//  std::cout << "     --hv_occlusion_th val:       Occlusion threshold (default " << hv_occlusion_th_ << ")" << std::endl;
//  std::cout << "     --hv_rad_clutter val:        Clutter radius (default " << hv_rad_clutter_ << ")" << std::endl;
//  std::cout << "     --hv_regularizer val:        Regularizer value (default " << hv_regularizer_ << ")" << std::endl;
//  std::cout << "     --hv_rad_normals val:        Normals radius (default " << hv_rad_normals_ << ")" << std::endl;
//  std::cout << "     --hv_detect_clutter val:     TRUE if clutter detect enabled (default " << hv_detect_clutter_ << ")" << std::endl << std::endl;
//}

///**
// * Parses Command Line Arguments (Argc,Argv)
// * @param argc
// * @param argv
// */
//void
//parseCommandLine (int argc,
//                  char *argv[])
//{
//  //Show help
//  if (pcl::console::find_switch (argc, argv, "-h"))
//  {
//    showHelp (argv[0]);
//    exit (0);
//  }

//  //Model & scene filenames
//  std::vector<int> filenames;
//  filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
//  if (filenames.size () != 2)
//  {
//    std::cout << "Filenames missing.\n";
//    showHelp (argv[0]);
//    exit (-1);
//  }

//  model_filename_ = argv[filenames[0]];
//  scene_filename_ = argv[filenames[1]];

//  //Program behavior
//  if (pcl::console::find_switch (argc, argv, "-k"))
//  {
//    show_keypoints_ = true;
//  }

//  std::string used_algorithm;
//  if (pcl::console::parse_argument (argc, argv, "--algorithm", used_algorithm) != -1)
//  {
//    if (used_algorithm.compare ("Hough") == 0)
//    {
//      use_hough_ = true;
//    }
//    else if (used_algorithm.compare ("GC") == 0)
//    {
//      use_hough_ = false;
//    }
//    else
//    {
//      std::cout << "Wrong algorithm name.\n";
//      showHelp (argv[0]);
//      exit (-1);
//    }
//  }

//  //General parameters
//  pcl::console::parse_argument (argc, argv, "--model_ss", model_ss_);
//  pcl::console::parse_argument (argc, argv, "--scene_ss", scene_ss_);
//  pcl::console::parse_argument (argc, argv, "--rf_rad", rf_rad_);
//  pcl::console::parse_argument (argc, argv, "--descr_rad", descr_rad_);
//  pcl::console::parse_argument (argc, argv, "--cg_size", cg_size_);
//  pcl::console::parse_argument (argc, argv, "--cg_thresh", cg_thresh_);
//  pcl::console::parse_argument (argc, argv, "--icp_max_iter", icp_max_iter_);
//  pcl::console::parse_argument (argc, argv, "--icp_corr_distance", icp_corr_distance_);
//  pcl::console::parse_argument (argc, argv, "--hv_clutter_reg", hv_clutter_reg_);
//  pcl::console::parse_argument (argc, argv, "--hv_inlier_th", hv_inlier_th_);
//  pcl::console::parse_argument (argc, argv, "--hv_occlusion_th", hv_occlusion_th_);
//  pcl::console::parse_argument (argc, argv, "--hv_rad_clutter", hv_rad_clutter_);
//  pcl::console::parse_argument (argc, argv, "--hv_regularizer", hv_regularizer_);
//  pcl::console::parse_argument (argc, argv, "--hv_rad_normals", hv_rad_normals_);
//  pcl::console::parse_argument (argc, argv, "--hv_detect_clutter", hv_detect_clutter_);
//}

//int
//main (int argc,
//      char *argv[])
//{
//  parseCommandLine (argc, argv);

//  pcl::PointCloud<PointType>::Ptr model (new pcl::PointCloud<PointType> ());
//  pcl::PointCloud<PointType>::Ptr model_keypoints (new pcl::PointCloud<PointType> ());
//  pcl::PointCloud<PointType>::Ptr scene (new pcl::PointCloud<PointType> ());
//  pcl::PointCloud<PointType>::Ptr scene_keypoints (new pcl::PointCloud<PointType> ());
//  pcl::PointCloud<NormalType>::Ptr model_normals (new pcl::PointCloud<NormalType> ());
//  pcl::PointCloud<NormalType>::Ptr scene_normals (new pcl::PointCloud<NormalType> ());
//  pcl::PointCloud<DescriptorType>::Ptr model_descriptors (new pcl::PointCloud<DescriptorType> ());
//  pcl::PointCloud<DescriptorType>::Ptr scene_descriptors (new pcl::PointCloud<DescriptorType> ());

//  /**
//   * Load Clouds
//   */
//  if (pcl::io::loadPCDFile (model_filename_, *model) < 0)
//  {
//    std::cout << "Error loading model cloud." << std::endl;
//    showHelp (argv[0]);
//    return (-1);
//  }
//  if (pcl::io::loadPCDFile (scene_filename_, *scene) < 0)
//  {
//    std::cout << "Error loading scene cloud." << std::endl;
//    showHelp (argv[0]);
//    return (-1);
//  }

//  /**
//   * Compute Normals
//   */
//  pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
//  norm_est.setKSearch (10);
//  norm_est.setInputCloud (model);
//  norm_est.compute (*model_normals);

//  norm_est.setInputCloud (scene);
//  norm_est.compute (*scene_normals);

//  /**
//   *  Downsample Clouds to Extract keypoints
//   */
//  pcl::VoxelGrid<PointType> uniform_sampling;
//  uniform_sampling.setInputCloud (model);
//  uniform_sampling.setLeafSize (model_ss_,model_ss_,model_ss_);
//  uniform_sampling.filter (*model_keypoints);
//  std::cout << "Model total points: " << model->size () << "; Selected Keypoints: " << model_keypoints->size () << std::endl;

//  uniform_sampling.setInputCloud (scene);
//  uniform_sampling.setLeafSize (scene_ss_,scene_ss_,scene_ss_);
//  uniform_sampling.filter (*scene_keypoints);
//  std::cout << "Scene total points: " << scene->size () << "; Selected Keypoints: " << scene_keypoints->size () << std::endl;

//  /**
//   *  Compute Descriptor for keypoints
//   */
//  pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est;
//  descr_est.setRadiusSearch (descr_rad_);

//  descr_est.setInputCloud (model_keypoints);
//  descr_est.setInputNormals (model_normals);
//  descr_est.setSearchSurface (model);
//  descr_est.compute (*model_descriptors);

//  descr_est.setInputCloud (scene_keypoints);
//  descr_est.setInputNormals (scene_normals);
//  descr_est.setSearchSurface (scene);
//  descr_est.compute (*scene_descriptors);

//  /**
//   *  Find Model-Scene Correspondences with KdTree
//   */
//  pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());
//  pcl::KdTreeFLANN<DescriptorType> match_search;
//  match_search.setInputCloud (model_descriptors);
//  std::vector<int> model_good_keypoints_indices;
//  std::vector<int> scene_good_keypoints_indices;

//  for (size_t i = 0; i < scene_descriptors->size (); ++i)
//  {
//    std::vector<int> neigh_indices (1);
//    std::vector<float> neigh_sqr_dists (1);
//    if (!pcl_isfinite (scene_descriptors->at (i).descriptor[0]))  //skipping NaNs
//    {
//      continue;
//    }
//    int found_neighs = match_search.nearestKSearch (scene_descriptors->at (i), 1, neigh_indices, neigh_sqr_dists);
//    if (found_neighs == 1 && neigh_sqr_dists[0] < 0.25f)
//    {
//      pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
//      model_scene_corrs->push_back (corr);
//      model_good_keypoints_indices.push_back (corr.index_query);
//      scene_good_keypoints_indices.push_back (corr.index_match);
//    }
//  }
//  pcl::PointCloud<PointType>::Ptr model_good_kp (new pcl::PointCloud<PointType> ());
//  pcl::PointCloud<PointType>::Ptr scene_good_kp (new pcl::PointCloud<PointType> ());
//  pcl::copyPointCloud (*model_keypoints, model_good_keypoints_indices, *model_good_kp);
//  pcl::copyPointCloud (*scene_keypoints, scene_good_keypoints_indices, *scene_good_kp);

//  std::cout << "Correspondences found: " << model_scene_corrs->size () << std::endl;

//  /**
//   *  Clustering
//   */
//  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
//  std::vector < pcl::Correspondences > clustered_corrs;

//  if (use_hough_)
//  {
//    pcl::PointCloud<RFType>::Ptr model_rf (new pcl::PointCloud<RFType> ());
//    pcl::PointCloud<RFType>::Ptr scene_rf (new pcl::PointCloud<RFType> ());

//    pcl::BOARDLocalReferenceFrameEstimation<PointType, NormalType, RFType> rf_est;
//    rf_est.setFindHoles (true);
//    rf_est.setRadiusSearch (rf_rad_);

//    rf_est.setInputCloud (model_keypoints);
//    rf_est.setInputNormals (model_normals);
//    rf_est.setSearchSurface (model);
//    rf_est.compute (*model_rf);

//    rf_est.setInputCloud (scene_keypoints);
//    rf_est.setInputNormals (scene_normals);
//    rf_est.setSearchSurface (scene);
//    rf_est.compute (*scene_rf);

//    //  Clustering
//    pcl::Hough3DGrouping<PointType, PointType, RFType, RFType> clusterer;
//    clusterer.setHoughBinSize (cg_size_);
//    clusterer.setHoughThreshold (cg_thresh_);
//    clusterer.setUseInterpolation (true);
//    clusterer.setUseDistanceWeight (false);

//    clusterer.setInputCloud (model_keypoints);
//    clusterer.setInputRf (model_rf);
//    clusterer.setSceneCloud (scene_keypoints);
//    clusterer.setSceneRf (scene_rf);
//    clusterer.setModelSceneCorrespondences (model_scene_corrs);

//    clusterer.recognize (rototranslations, clustered_corrs);
//  }
//  else
//  {
//    pcl::GeometricConsistencyGrouping<PointType, PointType> gc_clusterer;
//    gc_clusterer.setGCSize (cg_size_);
//    gc_clusterer.setGCThreshold (cg_thresh_);

//    gc_clusterer.setInputCloud (model_keypoints);
//    gc_clusterer.setSceneCloud (scene_keypoints);
//    gc_clusterer.setModelSceneCorrespondences (model_scene_corrs);

//    gc_clusterer.recognize (rototranslations, clustered_corrs);
//  }

//  /**
//   * Stop if no instances
//   */
//  if (rototranslations.size () <= 0)
//  {
//    cout << "*** No instances found! ***" << endl;
//    return (0);
//  }
//  else
//  {
//    cout << "Recognized Instances: " << rototranslations.size () << endl << endl;
//  }

//  /**
//   * Generates clouds for each instances found
//   */
//  std::vector<pcl::PointCloud<PointType>::ConstPtr> instances;

//  for (size_t i = 0; i < rototranslations.size (); ++i)
//  {
//    pcl::PointCloud<PointType>::Ptr rotated_model (new pcl::PointCloud<PointType> ());
//    pcl::transformPointCloud (*model, *rotated_model, rototranslations[i]);
//    instances.push_back (rotated_model);
//  }

//  /**
//   * ICP
//   */
//  std::vector<pcl::PointCloud<PointType>::ConstPtr> registered_instances;
//  if (true)
//  {
//    cout << "--- ICP ---------" << endl;

//    for (size_t i = 0; i < rototranslations.size (); ++i)
//    {
//      pcl::IterativeClosestPoint<PointType, PointType> icp;
//      icp.setMaximumIterations (icp_max_iter_);
//      icp.setMaxCorrespondenceDistance (icp_corr_distance_);
//      icp.setInputTarget (scene);
//      icp.setInputSource (instances[i]);
//      pcl::PointCloud<PointType>::Ptr registered (new pcl::PointCloud<PointType>);
//      icp.align (*registered);
//      registered_instances.push_back (registered);
//      cout << "Instance " << i << " ";
//      if (icp.hasConverged ())
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

//  /**
//   * Hypothesis Verification
//   */
//  cout << "--- Hypotheses Verification ---" << endl;
//  std::vector<bool> hypotheses_mask;  // Mask Vector to identify positive hypotheses

//  pcl::GlobalHypothesesVerification<PointType, PointType> GoHv;

//  GoHv.setSceneCloud (scene);  // Scene Cloud
//  GoHv.addModels (registered_instances, true);  //Models to verify

//  GoHv.setInlierThreshold (hv_inlier_th_);
//  GoHv.setOcclusionThreshold (hv_occlusion_th_);
//  GoHv.setRegularizer (hv_regularizer_);
//  GoHv.setRadiusClutter (hv_rad_clutter_);
//  GoHv.setClutterRegularizer (hv_clutter_reg_);
//  GoHv.setDetectClutter (hv_detect_clutter_);
//  GoHv.setRadiusNormals (hv_rad_normals_);

//  GoHv.verify ();
//  GoHv.getMask (hypotheses_mask);  // i-element TRUE if hvModels[i] verifies hypotheses

//  for (int i = 0; i < hypotheses_mask.size (); i++)
//  {
//    if (hypotheses_mask[i])
//    {
//      cout << "Instance " << i << " is GOOD! <---" << endl;
//    }
//    else
//    {
//      cout << "Instance " << i << " is bad!" << endl;
//    }
//  }
//  cout << "-------------------------------" << endl;

//  /**
//   *  Visualization
//   */
//  pcl::visualization::PCLVisualizer viewer ("Hypotheses Verification");
//  viewer.addPointCloud (scene, "scene_cloud");

//  pcl::PointCloud<PointType>::Ptr off_scene_model (new pcl::PointCloud<PointType> ());
//  pcl::PointCloud<PointType>::Ptr off_scene_model_keypoints (new pcl::PointCloud<PointType> ());

//  pcl::PointCloud<PointType>::Ptr off_model_good_kp (new pcl::PointCloud<PointType> ());
//  pcl::transformPointCloud (*model, *off_scene_model, Eigen::Vector3f (-1, 0, 0), Eigen::Quaternionf (1, 0, 0, 0));
//  pcl::transformPointCloud (*model_keypoints, *off_scene_model_keypoints, Eigen::Vector3f (-1, 0, 0), Eigen::Quaternionf (1, 0, 0, 0));
//  pcl::transformPointCloud (*model_good_kp, *off_model_good_kp, Eigen::Vector3f (-1, 0, 0), Eigen::Quaternionf (1, 0, 0, 0));

//  if (show_keypoints_)
//  {
//    CloudStyle modelStyle = style_white;
//    pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_color_handler (off_scene_model, modelStyle.r, modelStyle.g, modelStyle.b);
//    viewer.addPointCloud (off_scene_model, off_scene_model_color_handler, "off_scene_model");
//    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, modelStyle.size, "off_scene_model");
//  }

//  if (show_keypoints_)
//  {
//    CloudStyle goodKeypointStyle = style_violet;
//    pcl::visualization::PointCloudColorHandlerCustom<PointType> model_good_keypoints_color_handler (off_model_good_kp, goodKeypointStyle.r, goodKeypointStyle.g,
//                                                                                                    goodKeypointStyle.b);
//    viewer.addPointCloud (off_model_good_kp, model_good_keypoints_color_handler, "model_good_keypoints");
//    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, goodKeypointStyle.size, "model_good_keypoints");

//    pcl::visualization::PointCloudColorHandlerCustom<PointType> scene_good_keypoints_color_handler (scene_good_kp, goodKeypointStyle.r, goodKeypointStyle.g,
//                                                                                                    goodKeypointStyle.b);
//    viewer.addPointCloud (scene_good_kp, scene_good_keypoints_color_handler, "scene_good_keypoints");
//    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, goodKeypointStyle.size, "scene_good_keypoints");
//  }

//  for (size_t i = 0; i < instances.size (); ++i)
//  {
//    std::stringstream ss_instance;
//    ss_instance << "instance_" << i;

//    CloudStyle clusterStyle = style_red;
//    pcl::visualization::PointCloudColorHandlerCustom<PointType> instance_color_handler (instances[i], clusterStyle.r, clusterStyle.g, clusterStyle.b);
//    viewer.addPointCloud (instances[i], instance_color_handler, ss_instance.str ());
//    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, clusterStyle.size, ss_instance.str ());

//    CloudStyle registeredStyles = hypotheses_mask[i] ? style_green : style_cyan;
//    ss_instance << "_registered" << endl;
//    pcl::visualization::PointCloudColorHandlerCustom<PointType> registered_instance_color_handler (registered_instances[i], registeredStyles.r,
//                                                                                                   registeredStyles.g, registeredStyles.b);
//    viewer.addPointCloud (registered_instances[i], registered_instance_color_handler, ss_instance.str ());
//    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, registeredStyles.size, ss_instance.str ());
//  }

//  while (!viewer.wasStopped ())
//  {
//    viewer.spinOnce ();
//  }

//  return (0);
//}


/////////******------------------Construct a concave or convex hull polygon for a plane model---------------------------------***/
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/concave_hull.h>

int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>),
                                      cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>),
                                      cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCDReader reader;

  reader.read ("/home/zb/BoZhan/PCL/devel/lib/pcl_filter/table_scene_mug_stereo_textured.pcd", *cloud);
  // Build a filter to remove spurious NaNs
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0, 1.1);
  pass.filter (*cloud_filtered);
  std::cerr << "PointCloud after filtering has: "
            << cloud_filtered->points.size () << " data points." << std::endl;

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);

  seg.setInputCloud (cloud_filtered);
  seg.segment (*inliers, *coefficients);
  std::cerr << "PointCloud after segmentation has: "
            << inliers->indices.size () << " inliers." << std::endl;

  // Project the model inliers
  pcl::ProjectInliers<pcl::PointXYZ> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);
  proj.setIndices (inliers);
  proj.setInputCloud (cloud_filtered);
  proj.setModelCoefficients (coefficients);
  proj.filter (*cloud_projected);
  std::cerr << "PointCloud after projection has: "
            << cloud_projected->points.size () << " data points." << std::endl;

  // Create a Concave Hull representation of the projected inliers
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ConcaveHull<pcl::PointXYZ> chull;
  chull.setInputCloud (cloud_projected);
  chull.setAlpha (0.1);
  chull.reconstruct (*cloud_hull);

  std::cerr << "Concave hull has: " << cloud_hull->points.size ()
            << " data points." << std::endl;

  pcl::PCDWriter writer;
  writer.write ("table_scene_mug_stereo_textured_hull.pcd", *cloud_hull, false);

  return (0);
}
