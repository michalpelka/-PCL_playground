
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
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_estimation_normal_shooting.h>
#include <pcl/registration/transformation_estimation_lm.h>
//
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/registration/correspondence_rejection_median_distance.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/correspondence_rejection_trimmed.h>
#include <pcl/registration/correspondence_rejection_var_trimmed.h>
using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;


template <typename PointSource, typename PointTarget, typename Scalar = float>
class myIterativeClosestPoint:public pcl::IterativeClosestPoint <PointSource, PointTarget, Scalar>
{
public:
	pcl::CorrespondencesPtr getcorrespondeces()
	{
		return this->correspondences_;
	}
};

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
    grid.setLeafSize (0.35, 0.35, 0.35);
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
  //norm_est.setKSearch (30);
  norm_est.setRadiusSearch(1);
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
  



   //pcl::registration::TransformationEstimationLM<PointNormalT, PointNormalT, float>::Ptr te (new pcl::registration::TransformationEstimationLM<PointNormalT, PointNormalT, float>);
  pcl::registration::TransformationEstimationSVD<PointNormalT, PointNormalT, float>::Ptr te (new pcl::registration::TransformationEstimationSVD<PointNormalT, PointNormalT, float>);
  pcl::registration::CorrespondenceEstimation<PointNormalT, PointNormalT, float>::Ptr cens (new pcl::registration:: CorrespondenceEstimation<PointNormalT, PointNormalT, float>);
   //pcl::registration::CorrespondenceEstimationNormalShooting<PointNormalT, PointNormalT, PointNormalT>::Ptr cens (new pcl::registration::CorrespondenceEstimationNormalShooting<PointNormalT, PointNormalT, PointNormalT>);
  //CorrespondenceEstimationNormalShooting<PointNormalT, PointNormalT, PointNormalT, float>::Ptr cens (new CorrespondenceEstimationNormalShooting<PointNormalT, PointNormalT, PointNormalT, float>);
  cens->setInputSource (points_with_normals_src);
  cens->setInputTarget (points_with_normals_tgt);
  //cens->setSourceNormals (src);

  pcl::registration::CorrespondenceRejectorOneToOne::Ptr cor_rej_o2o (new pcl::registration::CorrespondenceRejectorOneToOne);
  
  pcl::registration::CorrespondenceRejectorMedianDistance::Ptr cor_rej_med (new pcl::registration::CorrespondenceRejectorMedianDistance);
  cor_rej_med->setInputSource<pcl::PointNormal> (points_with_normals_src);
  cor_rej_med->setInputTarget<pcl::PointNormal> (points_with_normals_tgt);

  pcl::registration::CorrespondenceRejectorSampleConsensus<PointNormalT>::Ptr cor_rej_sac (new pcl::registration::CorrespondenceRejectorSampleConsensus<PointNormalT>);
  cor_rej_sac->setInputSource (points_with_normals_src);
  cor_rej_sac->setInputTarget (points_with_normals_tgt);
  cor_rej_sac->setInlierThreshold (0.05);
  cor_rej_sac->setMaximumIterations (10000);

  pcl::registration::CorrespondenceRejectorVarTrimmed::Ptr cor_rej_var (new pcl::registration::CorrespondenceRejectorVarTrimmed);
  cor_rej_var->setInputSource<pcl::PointNormal> (points_with_normals_src);
  cor_rej_var->setInputTarget<pcl::PointNormal> (points_with_normals_tgt);
  
  pcl::registration::CorrespondenceRejectorTrimmed::Ptr cor_rej_tri (new pcl::registration::CorrespondenceRejectorTrimmed);
  //


  //
  // Align
  myIterativeClosestPoint<PointNormalT, PointNormalT, float> reg;
  reg.setTransformationEpsilon (1e-10);
  // Set the maximum distance between two correspondences (src<->tgt) to 10cm
  // Note: adjust this based on the size of your datasets
  //reg.setMaxCorrespondenceDistance (0.1);  
  // Set the point representation
  reg.setPointRepresentation (boost::make_shared<const MyPointRepresentation> (point_representation));

  reg.setInputSource (points_with_normals_src);
  reg.setInputTarget (points_with_normals_tgt);

  //reg.setCorrespondenceEstimation (cens);
  //reg.setTransformationEstimation (te);
  //reg.addCorrespondenceRejector (cor_rej_o2o);
  //reg.addCorrespondenceRejector (cor_rej_var);
  //reg.addCorrespondenceRejector (cor_rej_med);
  //reg.addCorrespondenceRejector (cor_rej_tri);
  //reg.addCorrespondenceRejector (cor_rej_sac);

  reg.setMaxCorrespondenceDistance(0.1);
  //
  // Run the same optimization in a loop and visualize the results
  Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
  PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
  reg.setMaximumIterations (2);
  for (int i = 0; i < 25; ++i)
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
	pcl::CorrespondencesPtr corrs = reg.getcorrespondeces();
	p->removeAllShapes();
	for (int i=0; i< corrs->size(); i++)
	{
		std::stringstream ss;
		ss <<"scan"<<i;
		int ind = (*corrs)[i].index_match;
		int quer = (*corrs)[i].index_query;
		
		p->addLine<PointNormalT>((*points_with_normals_src)[quer],(*points_with_normals_src)[ind],ss.str(),vp_2);
	}
    // visualize current state
    showCloudsRight(points_with_normals_tgt, points_with_normals_src);
	PCL_INFO ("%d", reg.hasConverged ()); PCL_INFO (" with score: %f\n",  reg.getFitnessScore ());


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


///*
// * Software License Agreement (BSD License)
// *
// *  Point Cloud Library (PCL) - www.pointclouds.org
// *  Copyright (c) 2012-, Open Perception, Inc.
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
// * $Id: normal_estimation.cpp 6746 2012-08-08 01:20:30Z rusu $
// *
// */
//
//#include <iostream>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//#include <pcl/console/print.h>
//#include <pcl/console/time.h>
//#include <pcl/console/parse.h>
//#include <pcl/registration/icp.h>
//#include <pcl/registration/correspondence_estimation.h>
//#include <pcl/registration/correspondence_estimation_normal_shooting.h>
//#include <pcl/registration/transformation_estimation_lm.h>
//
//#include <pcl/registration/correspondence_rejection_one_to_one.h>
//#include <pcl/registration/correspondence_rejection_median_distance.h>
//#include <pcl/registration/correspondence_rejection_sample_consensus.h>
//#include <pcl/registration/correspondence_rejection_trimmed.h>
//#include <pcl/registration/correspondence_rejection_var_trimmed.h>
//
//using namespace pcl;
//using namespace pcl::io;
//using namespace pcl::console;
//using namespace pcl::registration;
//
//Eigen::Vector4f    translation;
//Eigen::Quaternionf orientation;
//
//void
//printHelp (int, char **argv)
//{
//  print_error ("Syntax is: %s input_source.pcd input_target.pcd output.pcd [optional_arguments]\n", argv[0]);
//}
//
//bool
//loadCloud (const std::string &filename, pcl::PCLPointCloud2 &cloud)
//{
//  TicToc tt;
//  print_highlight ("Loading "); print_value ("%s ", filename.c_str ());
//
//  tt.tic ();
//  if (loadPCDFile (filename, cloud, translation, orientation) < 0)
//    return (false);
//  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", cloud.width * cloud.height); print_info (" points]\n");
//  print_info ("Available dimensions: "); print_value ("%s\n", getFieldsList (cloud).c_str ());
//
//  return (true);
//}
//
//void
//compute (const pcl::PCLPointCloud2::ConstPtr &source,
//         const pcl::PCLPointCloud2::ConstPtr &target,
//         pcl::PCLPointCloud2 &transformed_source)
//{
//  // Convert data to PointCloud<T>
//  PointCloud<PointNormal>::Ptr src (new PointCloud<PointNormal>);
//  PointCloud<PointNormal>::Ptr tgt (new PointCloud<PointNormal>);
//  fromPCLPointCloud2 (*source, *src);
//  fromPCLPointCloud2 (*target, *tgt);
//
//  // Estimate
//  TicToc tt;
//  tt.tic ();
//  
//  print_highlight (stderr, "Computing ");
//
//#define Scalar double
////#define Scalar float
//
//  TransformationEstimationLM<PointNormal, PointNormal, Scalar>::Ptr te (new TransformationEstimationLM<PointNormal, PointNormal, Scalar>);
//  //TransformationEstimationSVD<PointNormal, PointNormal, Scalar>::Ptr te (new TransformationEstimationSVD<PointNormal, PointNormal, Scalar>);
//  CorrespondenceEstimation<PointNormal, PointNormal, double>::Ptr cens (new CorrespondenceEstimation<PointNormal, PointNormal, double>);
//  //CorrespondenceEstimationNormalShooting<PointNormal, PointNormal, PointNormal>::Ptr cens (new CorrespondenceEstimationNormalShooting<PointNormal, PointNormal, PointNormal>);
//  //CorrespondenceEstimationNormalShooting<PointNormal, PointNormal, PointNormal, double>::Ptr cens (new CorrespondenceEstimationNormalShooting<PointNormal, PointNormal, PointNormal, double>);
//  cens->setInputSource (src);
//  cens->setInputTarget (tgt);
//  //cens->setSourceNormals (src);
//
//  CorrespondenceRejectorOneToOne::Ptr cor_rej_o2o (new CorrespondenceRejectorOneToOne);
//  
//  CorrespondenceRejectorMedianDistance::Ptr cor_rej_med (new CorrespondenceRejectorMedianDistance);
//  cor_rej_med->setInputSource<PointNormal> (src);
//  cor_rej_med->setInputTarget<PointNormal> (tgt);
//
//  CorrespondenceRejectorSampleConsensus<PointNormal>::Ptr cor_rej_sac (new CorrespondenceRejectorSampleConsensus<PointNormal>);
//  cor_rej_sac->setInputSource (src);
//  cor_rej_sac->setInputTarget (tgt);
//  cor_rej_sac->setInlierThreshold (0.005);
//  cor_rej_sac->setMaximumIterations (10000);
//
//  CorrespondenceRejectorVarTrimmed::Ptr cor_rej_var (new CorrespondenceRejectorVarTrimmed);
//  cor_rej_var->setInputSource<PointNormal> (src);
//  cor_rej_var->setInputTarget<PointNormal> (tgt);
//  
//  CorrespondenceRejectorTrimmed::Ptr cor_rej_tri (new CorrespondenceRejectorTrimmed);
//  
//  print_highlight (stderr, "ICP ");
//
//  IterativeClosestPoint<PointNormal, PointNormal, Scalar> icp;
//  icp.setCorrespondenceEstimation (cens);
//  icp.setTransformationEstimation (te);
//  icp.addCorrespondenceRejector (cor_rej_o2o);
//  //icp.addCorrespondenceRejector (cor_rej_var);
//  //icp.addCorrespondenceRejector (cor_rej_med);
//  //icp.addCorrespondenceRejector (cor_rej_tri);
//  //icp.addCorrespondenceRejector (cor_rej_sac);
//  
//  icp.setInputSource (src);
//  icp.setInputTarget (tgt);
//  icp.setMaximumIterations (100);
//  icp.setTransformationEpsilon (1e-10);
//  PointCloud<PointNormal> output;
//  icp.align (output);
//
//  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", output.width * output.height); print_info (" points], has converged: ");
//  print_value ("%d", icp.hasConverged ()); print_info (" with score: %f\n",  icp.getFitnessScore ());
//  Eigen::Matrix4d transformation = icp.getFinalTransformation ();
//  //Eigen::Matrix4f transformation = icp.getFinalTransformation ();
//  PCL_DEBUG ("Transformation is:\n\t%5f\t%5f\t%5f\t%5f\n\t%5f\t%5f\t%5f\t%5f\n\t%5f\t%5f\t%5f\t%5f\n\t%5f\t%5f\t%5f\t%5f\n", 
//      transformation (0, 0), transformation (0, 1), transformation (0, 2), transformation (0, 3),
//      transformation (1, 0), transformation (1, 1), transformation (1, 2), transformation (1, 3),
//      transformation (2, 0), transformation (2, 1), transformation (2, 2), transformation (2, 3),
//      transformation (3, 0), transformation (3, 1), transformation (3, 2), transformation (3, 3));
//
//  // Convert data back
//  pcl::PCLPointCloud2 output_source;
//  toPCLPointCloud2 (output, output_source);
//  concatenateFields (*source, output_source, transformed_source);
//}
//
//void
//saveCloud (const std::string &filename, const pcl::PCLPointCloud2 &output)
//{
//  TicToc tt;
//  tt.tic ();
//
//  print_highlight ("Saving "); print_value ("%s ", filename.c_str ());
//
//  PCDWriter w;
//  //w.writeBinaryCompressed (filename, output, translation, orientation);
//  w.writeASCII (filename, output, translation, orientation);
//  
//  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", output.width * output.height); print_info (" points]\n");
//}
//
//int
//main (int argc, char** argv)
//{
//  print_info ("Estimate a rigid transformation using IterativeClosestPoint. For more information, use: %s -h\n", argv[0]);
//
//  if (argc < 3)
//  {
//    printHelp (argc, argv);
//    return (-1);
//  }
//  
//  bool debug = false;
//  pcl::console::parse_argument (argc, argv, "-debug", debug);
//  if (debug)
//    pcl::console::setVerbosityLevel (pcl::console::L_DEBUG);
//
//  // Parse the command line arguments for .pcd files
//  std::vector<int> p_file_indices;
//  p_file_indices = parse_file_extension_argument (argc, argv, ".pcd");
//  if (p_file_indices.size () != 3)
//  {
//    print_error ("Need two input PCD files (source, target) and one output PCD file to continue.\n");
//    return (-1);
//  }
//
//  // Load the input files
//  pcl::PCLPointCloud2::Ptr src (new pcl::PCLPointCloud2);
//  if (!loadCloud (argv[p_file_indices[0]], *src)) return (-1);
//  pcl::PCLPointCloud2::Ptr tgt (new pcl::PCLPointCloud2);
//  if (!loadCloud (argv[p_file_indices[1]], *tgt)) return (-1);
//
//  // Perform the feature estimation
//  pcl::PCLPointCloud2 output;
//  compute (src, tgt, output);
//
//  // Save into the output file
//  saveCloud (argv[p_file_indices[2]], output);
//
//  return (0);
//}