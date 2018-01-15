///*-------------------------------example 1------------------------------------------*/

///* \author Geoffrey Biggs */


//#include <iostream>

//#include <boost/thread/thread.hpp>
//#include <pcl/common/common_headers.h>
//#include <pcl/features/normal_3d.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/console/parse.h>

//// --------------
//// -----Help-----
//// --------------
//void
//printUsage (const char* progName)
//{
//  std::cout << "\n\nUsage: "<<progName<<" [options]\n\n"
//            << "Options:\n"
//            << "-------------------------------------------\n"
//            << "-h           this help\n"
//            << "-s           Simple visualisation example\n"
//            << "-r           RGB colour visualisation example\n"
//            << "-c           Custom colour visualisation example\n"
//            << "-n           Normals visualisation example\n"
//            << "-a           Shapes visualisation example\n"
//            << "-v           Viewports example\n"
//            << "-i           Interaction Customization example\n"
//            << "\n\n";
//}


//boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
//{
//  // --------------------------------------------
//  // -----Open 3D viewer and add point cloud-----
//  // --------------------------------------------
//  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
//  viewer->setBackgroundColor (0, 0, 0);
//  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
//  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
//  viewer->addCoordinateSystem (1.0);
//  viewer->initCameraParameters ();
//  return (viewer);
//}


//boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
//{
//  // --------------------------------------------
//  // -----Open 3D viewer and add point cloud-----
//  // --------------------------------------------
//  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
//  viewer->setBackgroundColor (0, 0, 0);
//  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
//  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
//  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
//  viewer->addCoordinateSystem (1.0);
//  viewer->initCameraParameters ();
//  return (viewer);
//}


//boost::shared_ptr<pcl::visualization::PCLVisualizer> customColourVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
//{
//  // --------------------------------------------
//  // -----Open 3D viewer and add point cloud-----
//  // --------------------------------------------
//  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
//  viewer->setBackgroundColor (0, 0, 0);
//  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 255, 0);
//  viewer->addPointCloud<pcl::PointXYZ> (cloud, single_color, "sample cloud");
//  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
//  viewer->addCoordinateSystem (1.0);
//  viewer->initCameraParameters ();
//  return (viewer);
//}


//boost::shared_ptr<pcl::visualization::PCLVisualizer> normalsVis (
//    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals)
//{
//  // --------------------------------------------------------
//  // -----Open 3D viewer and add point cloud and normals-----
//  // --------------------------------------------------------
//  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
//  viewer->setBackgroundColor (0, 0, 0);
//  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
//  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
//  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
//  viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, normals, 10, 0.05, "normals");
//  viewer->addCoordinateSystem (1.0);
//  viewer->initCameraParameters ();
//  return (viewer);
//}


//boost::shared_ptr<pcl::visualization::PCLVisualizer> shapesVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
//{
//  // --------------------------------------------
//  // -----Open 3D viewer and add point cloud-----
//  // --------------------------------------------
//  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
//  viewer->setBackgroundColor (0, 0, 0);
//  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
//  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
//  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
//  viewer->addCoordinateSystem (1.0);
//  viewer->initCameraParameters ();

//  //------------------------------------
//  //-----Add shapes at cloud points-----
//  //------------------------------------
//  viewer->addLine<pcl::PointXYZRGB> (cloud->points[0],
//                                     cloud->points[cloud->size() - 1], "line");
//  viewer->addSphere (cloud->points[0], 0.2, 0.5, 0.5, 0.0, "sphere");

//  //---------------------------------------
//  //-----Add shapes at other locations-----
//  //---------------------------------------
//  pcl::ModelCoefficients coeffs;
//  coeffs.values.push_back (0.0);
//  coeffs.values.push_back (0.0);
//  coeffs.values.push_back (1.0);
//  coeffs.values.push_back (0.0);
//  viewer->addPlane (coeffs, "plane");
//  coeffs.values.clear ();
//  coeffs.values.push_back (0.3);
//  coeffs.values.push_back (0.3);
//  coeffs.values.push_back (0.0);
//  coeffs.values.push_back (0.0);
//  coeffs.values.push_back (1.0);
//  coeffs.values.push_back (0.0);
//  coeffs.values.push_back (5.0);
//  viewer->addCone (coeffs, "cone");

//  return (viewer);
//}


//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewportsVis (
//    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals1, pcl::PointCloud<pcl::Normal>::ConstPtr normals2)
//{
//  // --------------------------------------------------------
//  // -----Open 3D viewer and add point cloud and normals-----
//  // --------------------------------------------------------
//  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
//  viewer->initCameraParameters ();

//  int v1(0);
//  viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
//  viewer->setBackgroundColor (0, 0, 0, v1);
//  viewer->addText("Radius: 0.01", 10, 10, "v1 text", v1);
//  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
//  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud1", v1);

//  int v2(0);
//  viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
//  viewer->setBackgroundColor (0.3, 0.3, 0.3, v2);
//  viewer->addText("Radius: 0.1", 10, 10, "v2 text", v2);
//  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(cloud, 0, 255, 0);
//  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, single_color, "sample cloud2", v2);

//  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud1");
//  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud2");
//  viewer->addCoordinateSystem (1.0);

//  viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, normals1, 10, 0.05, "normals1", v1);
//  viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, normals2, 10, 0.05, "normals2", v2);

//  return (viewer);
//}


//unsigned int text_id = 0;
//void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
//                            void* viewer_void)
//{
//  pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
//  if (event.getKeySym () == "r" && event.keyDown ())
//  {
//    std::cout << "r was pressed => removing all text" << std::endl;

//    char str[512];
//    for (unsigned int i = 0; i < text_id; ++i)
//    {
//      sprintf (str, "text#%03d", i);
//      viewer->removeShape (str);
//    }
//    text_id = 0;
//  }
//}

//void mouseEventOccurred (const pcl::visualization::MouseEvent &event,
//                         void* viewer_void)
//{
//  pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
//  if (event.getButton () == pcl::visualization::MouseEvent::LeftButton &&
//      event.getType () == pcl::visualization::MouseEvent::MouseButtonRelease)
//  {
//    std::cout << "Left mouse button released at position (" << event.getX () << ", " << event.getY () << ")" << std::endl;

//    char str[512];
//    sprintf (str, "text#%03d", text_id ++);
//    viewer->addText ("clicked here", event.getX (), event.getY (), str);
//  }
//}

//boost::shared_ptr<pcl::visualization::PCLVisualizer> interactionCustomizationVis ()
//{
//  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
//  viewer->setBackgroundColor (0, 0, 0);
//  viewer->addCoordinateSystem (1.0);

//  viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)viewer.get ());
//  viewer->registerMouseCallback (mouseEventOccurred, (void*)viewer.get ());

//  return (viewer);
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
//  bool simple(false), rgb(false), custom_c(false), normals(false),
//    shapes(false), viewports(false), interaction_customization(false);
//  if (pcl::console::find_argument (argc, argv, "-s") >= 0)
//  {
//    simple = true;
//    std::cout << "Simple visualisation example\n";
//  }
//  else if (pcl::console::find_argument (argc, argv, "-c") >= 0)
//  {
//    custom_c = true;
//    std::cout << "Custom colour visualisation example\n";
//  }
//  else if (pcl::console::find_argument (argc, argv, "-r") >= 0)
//  {
//    rgb = true;
//    std::cout << "RGB colour visualisation example\n";
//  }
//  else if (pcl::console::find_argument (argc, argv, "-n") >= 0)
//  {
//    normals = true;
//    std::cout << "Normals visualisation example\n";
//  }
//  else if (pcl::console::find_argument (argc, argv, "-a") >= 0)
//  {
//    shapes = true;
//    std::cout << "Shapes visualisation example\n";
//  }
//  else if (pcl::console::find_argument (argc, argv, "-v") >= 0)
//  {
//    viewports = true;
//    std::cout << "Viewports example\n";
//  }
//  else if (pcl::console::find_argument (argc, argv, "-i") >= 0)
//  {
//    interaction_customization = true;
//    std::cout << "Interaction Customization example\n";
//  }
//  else
//  {
//    printUsage (argv[0]);
//    return 0;
//  }

//  // ------------------------------------
//  // -----Create example point cloud-----
//  // ------------------------------------
//  pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
//  pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
//  std::cout << "Genarating example point clouds.\n\n";
//  // We're going to make an ellipse extruded along the z-axis. The colour for
//  // the XYZRGB cloud will gradually go from red to green to blue.
//  uint8_t r(255), g(15), b(15);
//  for (float z(-1.0); z <= 1.0; z += 0.05)
//  {
//    for (float angle(0.0); angle <= 360.0; angle += 5.0)
//    {
//      pcl::PointXYZ basic_point;
//      basic_point.x = 0.5 * cosf (pcl::deg2rad(angle));
//      basic_point.y = sinf (pcl::deg2rad(angle));
//      basic_point.z = z;
//      basic_cloud_ptr->points.push_back(basic_point);

//      pcl::PointXYZRGB point;
//      point.x = basic_point.x;
//      point.y = basic_point.y;
//      point.z = basic_point.z;
//      uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
//              static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
//      point.rgb = *reinterpret_cast<float*>(&rgb);
//      point_cloud_ptr->points.push_back (point);
//    }
//    if (z < 0.0)
//    {
//      r -= 12;
//      g += 12;
//    }
//    else
//    {
//      g -= 12;
//      b += 12;
//    }
//  }
//  basic_cloud_ptr->width = (int) basic_cloud_ptr->points.size ();
//  basic_cloud_ptr->height = 1;
//  point_cloud_ptr->width = (int) point_cloud_ptr->points.size ();
//  point_cloud_ptr->height = 1;

//  // ----------------------------------------------------------------
//  // -----Calculate surface normals with a search radius of 0.05-----
//  // ----------------------------------------------------------------
//  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
//  ne.setInputCloud (point_cloud_ptr);
//  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
//  ne.setSearchMethod (tree);
//  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals1 (new pcl::PointCloud<pcl::Normal>);
//  ne.setRadiusSearch (0.05);
//  ne.compute (*cloud_normals1);

//  // ---------------------------------------------------------------
//  // -----Calculate surface normals with a search radius of 0.1-----
//  // ---------------------------------------------------------------
//  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
//  ne.setRadiusSearch (0.1);
//  ne.compute (*cloud_normals2);

//  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
//  if (simple)
//  {
//    viewer = simpleVis(basic_cloud_ptr);
//  }
//  else if (rgb)
//  {
//    viewer = rgbVis(point_cloud_ptr);
//  }
//  else if (custom_c)
//  {
//    viewer = customColourVis(basic_cloud_ptr);
//  }
//  else if (normals)
//  {
//    viewer = normalsVis(point_cloud_ptr, cloud_normals2);
//  }
//  else if (shapes)
//  {
//    viewer = shapesVis(point_cloud_ptr);
//  }
//  else if (viewports)
//  {
//    viewer = viewportsVis(point_cloud_ptr, cloud_normals1, cloud_normals2);
//  }
//  else if (interaction_customization)
//  {
//    viewer = interactionCustomizationVis();
//  }

//  //--------------------
//  // -----Main loop-----
//  //--------------------
//  while (!viewer->wasStopped ())
//  {
//    viewer->spinOnce (100);
//    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
//  }
//}


///*-------------------------------example 2------------------------------------------*/

/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

/* \author Radu Bogdan Rusu
 * adaptation Raphael Favier*/

#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/io/pcd_io.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include <pcl/visualization/pcl_visualizer.h>

using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;

//convenient typedefs
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

// This is a tutorial so we can afford having global variables
  //our visualizer
  pcl::visualization::PCLVisualizer *p;
  //its left and right viewports
  int vp_1, vp_2;

//convenient structure to handle our pointclouds
struct PCD
{
  PointCloud::Ptr cloud;
  std::string f_name;

  PCD() : cloud (new PointCloud) {};
};

struct PCDComparator
{
  bool operator () (const PCD& p1, const PCD& p2)
  {
    return (p1.f_name < p2.f_name);
  }
};


// Define a new point representation for < x, y, z, curvature >
class MyPointRepresentation : public pcl::PointRepresentation <PointNormalT>
{
  using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
public:
  MyPointRepresentation ()
  {
    // Define the number of dimensions
    nr_dimensions_ = 4;
  }

  // Override the copyToFloatArray method to define our feature vector
  virtual void copyToFloatArray (const PointNormalT &p, float * out) const
  {
    // < x, y, z, curvature >
    out[0] = p.x;
    out[1] = p.y;
    out[2] = p.z;
    out[3] = p.curvature;
  }
};


////////////////////////////////////////////////////////////////////////////////
/** \brief Display source and target on the first viewport of the visualizer
 *
 */
void showCloudsLeft(const PointCloud::Ptr cloud_target, const PointCloud::Ptr cloud_source)
{
  p->removePointCloud ("vp1_target");
  p->removePointCloud ("vp1_source");

  PointCloudColorHandlerCustom<PointT> tgt_h (cloud_target, 0, 255, 0);
  PointCloudColorHandlerCustom<PointT> src_h (cloud_source, 255, 0, 0);
  p->addPointCloud (cloud_target, tgt_h, "vp1_target", vp_1);
  p->addPointCloud (cloud_source, src_h, "vp1_source", vp_1);

  PCL_INFO ("Press q to begin the registration.\n");
  p-> spin();
}


////////////////////////////////////////////////////////////////////////////////
/** \brief Display source and target on the second viewport of the visualizer
 *
 */
void showCloudsRight(const PointCloudWithNormals::Ptr cloud_target, const PointCloudWithNormals::Ptr cloud_source)
{
  p->removePointCloud ("source");
  p->removePointCloud ("target");


  PointCloudColorHandlerGenericField<PointNormalT> tgt_color_handler (cloud_target, "curvature");
  if (!tgt_color_handler.isCapable ())
      PCL_WARN ("Cannot create curvature color handler!");

  PointCloudColorHandlerGenericField<PointNormalT> src_color_handler (cloud_source, "curvature");
  if (!src_color_handler.isCapable ())
      PCL_WARN ("Cannot create curvature color handler!");


  p->addPointCloud (cloud_target, tgt_color_handler, "target", vp_2);
  p->addPointCloud (cloud_source, src_color_handler, "source", vp_2);

  p->spinOnce();
}

////////////////////////////////////////////////////////////////////////////////
/** \brief Load a set of PCD files that we want to register together
  * \param argc the number of arguments (pass from main ())
  * \param argv the actual command line arguments (pass from main ())
  * \param models the resultant vector of point cloud datasets
  */
void loadData (int argc, char **argv, std::vector<PCD, Eigen::aligned_allocator<PCD> > &models)
{
  std::string extension (".pcd");
  // Suppose the first argument is the actual test model
  for (int i = 1; i < argc; i++)
  {
    std::string fname = std::string (argv[i]);
    // Needs to be at least 5: .plot
    if (fname.size () <= extension.size ())
      continue;

    std::transform (fname.begin (), fname.end (), fname.begin (), (int(*)(int))tolower);

    //check that the argument is a pcd file
    if (fname.compare (fname.size () - extension.size (), extension.size (), extension) == 0)
    {
      // Load the cloud and saves it into the global list of models
      PCD m;
      m.f_name = argv[i];
      pcl::io::loadPCDFile (argv[i], *m.cloud);
      //remove NAN points from the cloud
      std::vector<int> indices;
      pcl::removeNaNFromPointCloud(*m.cloud,*m.cloud, indices);

      models.push_back (m);
    }
  }
}


////////////////////////////////////////////////////////////////////////////////
/** \brief Align a pair of PointCloud datasets and return the result
  * \param cloud_src the source PointCloud
  * \param cloud_tgt the target PointCloud
  * \param output the resultant aligned source PointCloud
  * \param final_transform the resultant transform between source and target
  */
void pairAlign (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample = false)
{
  //
  // Downsample for consistency and speed
  // \note enable this for large datasets
  PointCloud::Ptr src (new PointCloud);
  PointCloud::Ptr tgt (new PointCloud);
  pcl::VoxelGrid<PointT> grid;
  if (downsample)
  {
    grid.setLeafSize (0.05, 0.05, 0.05);
    grid.setInputCloud (cloud_src);
    grid.filter (*src);

    grid.setInputCloud (cloud_tgt);
    grid.filter (*tgt);
  }
  else
  {
    src = cloud_src;
    tgt = cloud_tgt;
  }


  // Compute surface normals and curvature
  PointCloudWithNormals::Ptr points_with_normals_src (new PointCloudWithNormals);
  PointCloudWithNormals::Ptr points_with_normals_tgt (new PointCloudWithNormals);

  pcl::NormalEstimation<PointT, PointNormalT> norm_est;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  norm_est.setSearchMethod (tree);
  norm_est.setKSearch (30);

  norm_est.setInputCloud (src);
  norm_est.compute (*points_with_normals_src);
  pcl::copyPointCloud (*src, *points_with_normals_src);

  norm_est.setInputCloud (tgt);
  norm_est.compute (*points_with_normals_tgt);
  pcl::copyPointCloud (*tgt, *points_with_normals_tgt);

  //
  // Instantiate our custom point representation (defined above) ...
  MyPointRepresentation point_representation;
  // ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
  float alpha[4] = {1.0, 1.0, 1.0, 1.0};
  point_representation.setRescaleValues (alpha);

  //
  // Align
  pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
  reg.setTransformationEpsilon (1e-6);
  // Set the maximum distance between two correspondences (src<->tgt) to 10cm
  // Note: adjust this based on the size of your datasets
  reg.setMaxCorrespondenceDistance (0.1);
  // Set the point representation
  reg.setPointRepresentation (boost::make_shared<const MyPointRepresentation> (point_representation));

  reg.setInputSource (points_with_normals_src);
  reg.setInputTarget (points_with_normals_tgt);



  //
  // Run the same optimization in a loop and visualize the results
  Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
  PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
  reg.setMaximumIterations (2);
  for (int i = 0; i < 30; ++i)
  {
    PCL_INFO ("Iteration Nr. %d.\n", i);

    // save cloud for visualization purpose
    points_with_normals_src = reg_result;

    // Estimate
    reg.setInputSource (points_with_normals_src);
    reg.align (*reg_result);

    //accumulate transformation between each Iteration
    Ti = reg.getFinalTransformation () * Ti;

    //if the difference between this transformation and the previous one
    //is smaller than the threshold, refine the process by reducing
    //the maximal correspondence distance
    if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
      reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.001);

    prev = reg.getLastIncrementalTransformation ();

    // visualize current state
    showCloudsRight(points_with_normals_tgt, points_with_normals_src);
  }

  //
  // Get the transformation from target to source
  targetToSource = Ti.inverse();

  //
  // Transform target back in source frame
  pcl::transformPointCloud (*cloud_tgt, *output, targetToSource);

  p->removePointCloud ("source");
  p->removePointCloud ("target");

  PointCloudColorHandlerCustom<PointT> cloud_tgt_h (output, 0, 255, 0);
  PointCloudColorHandlerCustom<PointT> cloud_src_h (cloud_src, 255, 0, 0);
  p->addPointCloud (output, cloud_tgt_h, "target", vp_2);
  p->addPointCloud (cloud_src, cloud_src_h, "source", vp_2);

  PCL_INFO ("Press q to continue the registration.\n");
  p->spin ();

  p->removePointCloud ("source");
  p->removePointCloud ("target");

  //add the source to the transformed target
  *output += *cloud_src;

  final_transform = targetToSource;
 }


/* ---[ */
int main (int argc, char** argv)
{
  // Load data
  std::vector<PCD, Eigen::aligned_allocator<PCD> > data;
  loadData (argc, argv, data);

  // Check user input
  if (data.empty ())
  {
    PCL_ERROR ("Syntax is: %s <source.pcd> <target.pcd> [*]", argv[0]);
    PCL_ERROR ("[*] - multiple files can be added. The registration results of (i, i+1) will be registered against (i+2), etc");
    return (-1);
  }
  PCL_INFO ("Loaded %d datasets.", (int)data.size ());

  // Create a PCLVisualizer object
  p = new pcl::visualization::PCLVisualizer (argc, argv, "Pairwise Incremental Registration example");
  p->createViewPort (0.0, 0, 0.5, 1.0, vp_1);
  p->createViewPort (0.5, 0, 1.0, 1.0, vp_2);

  PointCloud::Ptr result (new PointCloud), source, target;
  Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity (), pairTransform;

  for (size_t i = 1; i < data.size (); ++i)
  {
    source = data[i-1].cloud;
    target = data[i].cloud;

    // Add visualization data
    showCloudsLeft(source, target);

    PointCloud::Ptr temp (new PointCloud);
    PCL_INFO ("Aligning %s (%d) with %s (%d).\n", data[i-1].f_name.c_str (), source->points.size (), data[i].f_name.c_str (), target->points.size ());
    pairAlign (source, target, temp, pairTransform, true);

    //transform current pair into the global transform
    pcl::transformPointCloud (*temp, *result, GlobalTransform);

    //update the global transform
    GlobalTransform = GlobalTransform * pairTransform;

    //save aligned pair, transformed into the first cloud's frame
    std::stringstream ss;
    ss << i << ".pcd";
    pcl::io::savePCDFile (ss.str (), *result, true);

  }
}
/* ]--- */
