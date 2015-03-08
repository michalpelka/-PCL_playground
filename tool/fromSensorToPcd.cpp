#include <iostream>
#include <fstream>
#include "dataFramework\dataTransformations.hpp"
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <Eigen\Eigen>

dataTransformation tt;
std::string path="C:/immData_robot/";

int convert(std::string id)
{
	pcl::PointCloud<pcl::PointXYZ> pc;
	pcl::io::loadPCDFile<pcl::PointXYZ>(path+id+".pcd",pc);
	Eigen::Affine3f mat;
	mat.setIdentity();
	
	Eigen::Vector3f trans (pc.sensor_origin_.x(),pc.sensor_origin_.y(), pc.sensor_origin_.z());
	mat.translate(trans);
	mat.rotate(pc.sensor_orientation_);
	tt.setAffine(id, mat.matrix());
	tt.setPointcloudName(id, id+".pcd");
	tt.saveFile(path+"modelOdom.xml");
}

std::string generateId( int id)
{
	char txt[200];
	sprintf(txt,"scan%03d",id);
	return txt;
}
int main()
{
	for (int i=0; i< 100; i++)
	{
		convert(generateId(i));
	}
}