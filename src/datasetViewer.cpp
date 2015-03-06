
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "dataFramework\dataTransformations.hpp"

int main (int argc, char** argv)
{

		pcl::visualization::PCLVisualizer p;

	/// loadpointclouds
	
	if (argc !=2 && argc !=3)
	{
		std::cout << "USAGE:\n";
		std::cout << argv[0]<<" xml_model_file folder_with_pcd\n";
		return -1;
	}
	std::string model_file = argv[1];
	std::string pcd_path = "";
	if (argc == 3) pcd_path = argv[2];

	dataTransformation dSets;
	dSets.loadFile(model_file);

	std::vector<std::string> indices;
	dSets.getAllScansId(indices);

	for (int i=0; i< indices.size(); i++)
	{
		std::string fn;
		dSets.getPointcloudName(indices[i], fn);
		std::cout << indices[i]<<"\t"<<fn<<"\n";
	}
	p.addCoordinateSystem(5);
	for (int i=0; i < indices.size(); i++)
	{
		
		pcl::PointCloud<pcl::PointXYZ>::Ptr pc (new pcl::PointCloud<pcl::PointXYZ>);

	
		
		std::string fn;
		Eigen::Matrix4f transform;
		bool isOkFn = dSets.getPointcloudName(indices[i], fn);
		bool isOkTr = dSets.getAffine(indices[i], transform);

		if (isOkFn && isOkTr)
		{
			std::cout <<"============\n";
			std::cout <<"adding pc "<< indices[i]<<"\n";
			std::cout <<"with transform \n"<< transform<<"\n";
			pcl::io::loadPCDFile<pcl::PointXYZ>(fn, *pc);
			pc->sensor_origin_ = Eigen::Vector4f(0,0,0,0);
			pc->sensor_orientation_ = Eigen::Quaternionf::Identity();

			pcl::transformPointCloud(*pc,*pc, transform);
			p.addPointCloud(pc, indices[i]);
		}
		
		
	}



	p.spin();

	return 0;
}

