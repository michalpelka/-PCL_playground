
//haris






#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/filters/voxel_grid.h>

int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudNonSubsample (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ("scan20150222T111237 - Cloud.pcd", *cloudNonSubsample) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }

  pcl::VoxelGrid<pcl::PointXYZRGB> sor;
  sor.setInputCloud (cloudNonSubsample);
  sor.setLeafSize (0.1f, 0.1f, 0.1f);
  sor.filter (*cloud);

  std::cout <<"input cloud counbt "<< cloudNonSubsample->size() <<"\n";
  std::cout <<"input cloud filtered: "<< cloud->size() <<"\n";

  std::cout <<"readed\n";
  pcl::visualization::PCLVisualizer viewer("PCL Viewer");
  


  pcl::HarrisKeypoint3D <pcl::PointXYZRGB, pcl::PointXYZI> detector;
  pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints (new pcl::PointCloud<pcl::PointXYZI>);
  std::cout <<"computed1\n";
  detector.setNonMaxSupression (true);
  detector.setRadius(1.1);
  detector.setInputCloud (cloud);
  detector.setThreshold (1e-5);
  std::cout <<"computed2\n";
  detector.compute (*keypoints);
  
  for (auto it=  keypoints->begin(); it != keypoints->end(); it++)
  {
	  if (fabs(it->x)> 100 ||fabs(it->y)> 100 || fabs(it->z)> 100)
	  {
		  it->x = 0;
		  it->y = 0;
		  it->z = 0;

	  }
  }

  std::cout <<"computed "<<keypoints->size()<<"\n";
  
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> keypoints_color_handler (keypoints, 255, 255, 0);
  viewer.addPointCloud(keypoints,keypoints_color_handler, "keypoints");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints");
  viewer.addPointCloud(cloud, "inputCloud");


    while(!viewer.wasStopped ())
  {
  viewer.spinOnce ();
  }

  return (0);
}
