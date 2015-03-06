


#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/features/pfhrgb.h>
#include <pcl/filters/voxel_grid.h>



void normals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr c, pcl::PointCloud<pcl::Normal>::Ptr n)
{

	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimation;
	normal_estimation.setSearchMethod (pcl::search::Search<pcl::PointXYZRGB>::Ptr (new pcl::search::KdTree<pcl::PointXYZRGB>));
	normal_estimation.setRadiusSearch (0.2);
	normal_estimation.setInputCloud (c);
	normal_estimation.compute (*n);
	pcl::io::savePCDFileASCII("normals.pcd", *n);
}
int
main (int argc, char** argv)
{

	  pcl::visualization::PCLVisualizer viewer("PCL Viewer");
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints1 (new pcl::PointCloud<pcl::PointXYZI>);

  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints2 (new pcl::PointCloud<pcl::PointXYZI>);


  pcl::io::loadPCDFile<pcl::PointXYZRGB> ("1.pcd", *cloud1);
  pcl::io::loadPCDFile<pcl::PointXYZRGB> ("2.pcd", *cloud2);
  


  pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointXYZI> sift_detect1;
  pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointXYZI> sift_detect2;


  
  //sift_detect.setSearchMethod(pcl::search::KdTreeFlann<pcl::PointXYZRGB>::Ptr (new pcl:: "out.txt"<pcl::PointXYZRGB>));

  // Set the detection parameters
	sift_detect1.setScales (0.01, 3, 2);
	sift_detect1.setMinimumContrast (25);
	sift_detect1.setInputCloud(cloud1);
	sift_detect1.compute(*keypoints1);
  
	sift_detect2.setScales (0.01, 3, 2);
	sift_detect2.setMinimumContrast (25);
	sift_detect2.setInputCloud(cloud2);
	sift_detect2.compute(*keypoints2);
  
	
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> keypoint_color1 (keypoints1, 255, 0, 0);
    viewer.addPointCloud(keypoints1, keypoint_color1, "keypoints1");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints1");
	viewer.addPointCloud(cloud1, "inputCloud1");
	
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> keypoint_color2 (keypoints2, 0, 255, 0);
    viewer.addPointCloud(keypoints2, keypoint_color2, "keypoints2");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints2");
	viewer.addPointCloud(cloud2, "inputCloud2");

	std::cout <<"normals"<<"\n";

	pcl::PointCloud<pcl::Normal>::Ptr normals1 (new  pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::Normal>::Ptr normals2 (new  pcl::PointCloud<pcl::Normal>);

	normals(cloud1, normals1);
	normals(cloud2, normals2);


	pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr feaures1 (new pcl::PointCloud<pcl::PFHRGBSignature250>);
	pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr feaures2 (new pcl::PointCloud<pcl::PFHRGBSignature250>);


	pcl::PointCloud<pcl::PointXYZRGB>::Ptr kpts1(new pcl::PointCloud<pcl::PointXYZRGB>);
	kpts1->points.resize(keypoints1->points.size());
	pcl::copyPointCloud(*keypoints1, *kpts1);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr kpts2(new pcl::PointCloud<pcl::PointXYZRGB>);
	kpts1->points.resize(keypoints2->points.size());
	pcl::copyPointCloud(*keypoints2, *kpts2);



	pcl::Feature<pcl::PointXYZRGB, pcl::PFHRGBSignature250>::Ptr feature_extractor (new pcl::PFHRGBEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::PFHRGBSignature250>);
	feature_extractor->setKSearch(50);
	feature_extractor->setSearchSurface(cloud1);

	feature_extractor->setInputCloud(kpts1);
	feature_extractor->compute(*feaures1);

	feature_extractor->setInputCloud(kpts2);
	feature_extractor->compute(*feaures2);

	pcl::io::savePCDFileASCII("keypoints1.pcd", *keypoints1);
	pcl::io::savePCDFileASCII("keypoints2.pcd", *keypoints2);
	


	while(!viewer.wasStopped ())
	{
		viewer.spinOnce ();
	}

//	pcl::io::savePCDFileASCII("sift1.pcd", *keypoints1);
	
  return (0);
}
