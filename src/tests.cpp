#include <dataFramework\dataTransformations.hpp>
#include <Eigen/Eigen>
#include <iostream>
int main()
{

	dataTransformation tt;
	tt.setPointcloudName("scan000", "scan000.pcd");
	Eigen::Affine3f ff;
	ff=Eigen::Affine3f::Identity();
	ff.translation()<<0,0,99;
	ff.rotate(Eigen::AngleAxisf(3.14, Eigen::Vector3f::UnitZ()));
	tt.setAffine("scan000", ff.matrix());
	

	Eigen::Affine3f ff2;
	ff2=Eigen::Affine3f::Identity();
	ff2.translation()<<0,215,200;
	tt.setAffine("scan000", ff2.matrix());
	
	
	tt.setPointcloudName("scan000", "scan000.pcd");
	tt.setPointcloudName("scan001", "scan001.pcd");
	tt.setPointcloudName("scan002", "scan002.pcd");
	
	tt.saveFile("test.xml");




	dataTransformation tt2;
	tt.loadFile("test.xml");
	Eigen::Matrix4f matrix;
	bool isok = tt.getAffine("scan000", matrix);
	std::cout << "isok "<< isok<<"\n";
	std::cout << matrix;
	std::string pcName;
	isok = tt.getPointcloudName("scan000", pcName);

	std::cout<<pcName<<"\n"; 
}