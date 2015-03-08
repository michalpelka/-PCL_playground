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
	std::stringstream ss;
	ss<<"scan";
	if (id >=0 && id <10)
	{
		ss<<"00"<<id;
		return ss.str();
	}
	if (id >=10 && id <100)
	{
		ss<<"0"<<id;
		return ss.str();
	}
	if (id >=100 && id <1000)
	{
		ss<<id;
		return ss.str();
	}
}
int main()
{
for (int i=0; i< 100; i++)
{
	convert(generateId(i));
}


}