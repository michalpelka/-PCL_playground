#include <iostream>
#include <fstream>
#include "dataFramework\dataTransformations.hpp"
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <Eigen\Eigen>
dataTransformation tt;
std::string path="C:/parkingDATA_robot/";
int convert(std::string id)
{
	{
		std::cout <<"reading "<<id<<"\n";
		std::ifstream ff;
		ff.open((path+id+".3d").c_str());
		if (!ff.is_open()) 
		{
			std::cout <<"error reading "<<id<<"\n";
				return -1;
		}
		pcl::PointCloud<pcl::PointXYZ> pc;
		while (!ff.eof())
		{
			float x,y,z;
			ff>>x>>y>>z;
			float X,Y,Z;
			X = 0.001* x;
			Z = 0.001* y;
			Y =-0.001* z;
			pcl::PointXYZ xyz;
			xyz.x = X;
			xyz.y = Y;
			xyz.z = Z;

			pc.push_back(xyz);
		}
		pcl::io::savePCDFile(path+id+".pcd", pc);
		std::cout <<"3d ok\n";
		ff.close();
	}
	std::ifstream frame;
	frame.open(path+id+".pose");
	float _x,_y,_z;
	float _rX,_rY,_rZ;
	frame>>_x>>_y>>_z;
	frame>>_rX>>_rY>>_rZ;
	float x,y;
	float rotZ;
	x = 0.001*_x;
	y = -0.001*_z;
	rotZ = M_PI * (_rY/180.0);
	Eigen::Affine3f mat = Eigen::Affine3f::Identity();
	mat.translate(Eigen::Vector3f(x,y, 0));
	mat.rotate(Eigen::AngleAxisf(rotZ, Eigen::Vector3f::UnitZ()));
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