
/* \author Radu Bogdan Rusu
 * adaptation Raphael Favier*/

#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/io/pcd_io.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <vector>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/ndt.h>
#include <pcl/console/parse.h>

#include <pcl/registration/icp.h>
#include "configReader.hpp"
#include "dataFramework\dataTransformations.hpp"

bool display_help = false;
float filter_res = 0.2f;
double step_size = 0.1;
float ndt_res = 1.0f;
int iter = 35;
double trans_eps = 0.01;

using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;

//convenient typedefs
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;


pcl::PointCloud<PointT> metascan;

std::vector<std::string> pcNames;
pcl::visualization::PCLVisualizer p;

Eigen::Affine3f lastGlobalOdom;
Eigen::Affine3f lastFit;
Eigen::Affine3f currentFitCandidate;
Eigen::Affine3f currentlyAssignedGlobalOdom;
Eigen::Affine3f currentInitialFit;



int currentPointcloud =0;
pcl::PointCloud<PointT>::Ptr currentlyRegisteredPc;
bool registration_accepted = false;
configReader myCFG;
std::vector<std::string> msg;

void addMessage (std::string text)
{
	msg.insert(msg.begin(),text);
	for (int i=0; i< 25; i++)
	{
		std::stringstream ss;
		ss<<"text"<<i;
		if (msg[i].size()==0)msg[i]=".."; 
		p.updateText(msg[i],10, 20+i*15, 10,0,1,0, ss.str());
		p.spinOnce();
	}
}
bool registerICP(pcl::PointCloud<PointT> &metascan, pcl::PointCloud<PointT> &scan, Eigen::Affine3f &metascanToScan)
{
	std::cout <<"invoking ICP\n";
	
    pcl::IterativeClosestPoint<PointT, PointT> *icp;

    icp = new pcl::IterativeClosestPoint<PointT, PointT>();

    icp->setMaximumIterations (200);
    icp->setMaxCorrespondenceDistance (1.0);
    icp->setRANSACOutlierRejectionThreshold (0.1);

    //icp->setInputTarget (model);
	icp->setInputTarget (metascan.makeShared());
    icp->setInputSource (scan.makeShared());

    pcl::PointCloud<PointT>::Ptr tmp (new pcl::PointCloud<PointT>);
    icp->align (*tmp);
	std::cout << icp->getFinalTransformation () << std::endl;
	metascanToScan = icp->getFinalTransformation();
}
bool registerNDT(pcl::PointCloud<PointT> &metascan, pcl::PointCloud<PointT> &scan, Eigen::Affine3f &metascanToScan)
{
	std::cout <<"invoking NDT\n";
	pcl::NormalDistributionsTransform<PointT, PointT> * ndt = new pcl::NormalDistributionsTransform<PointT, PointT>();

	ndt->setMaximumIterations (myCFG.ndt_iter);
	ndt->setResolution (myCFG.ndt_res);
	ndt->setStepSize (myCFG.ndt_step_size);
	ndt->setTransformationEpsilon (myCFG.ndt_trans_eps);
	ndt->setInputTarget (metascan.makeShared());
	ndt->setInputSource (scan.makeShared());
	pcl::PointCloud<PointT>::Ptr tmp (new pcl::PointCloud<PointT>);
    ndt->align (*tmp);
	
    std::cout << ndt->getFinalTransformation () << std::endl;
	metascanToScan = ndt->getFinalTransformation();
}
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void)
{
	if (event.getKeySym()=="l" && event.keyUp())
	{
		currentPointcloud++;
		if (currentPointcloud < pcNames.size())
		{

			std::cout << "loading pointcloud "<<pcNames[currentPointcloud]<<"\n";
			currentlyRegisteredPc = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
			pcl::io::loadPCDFile(pcNames[currentPointcloud], *currentlyRegisteredPc);
			
			currentlyAssignedGlobalOdom.setIdentity();
			currentlyAssignedGlobalOdom.translation()<< currentlyRegisteredPc->sensor_origin_[0], currentlyRegisteredPc->sensor_origin_[1], currentlyRegisteredPc->sensor_origin_[2];
			currentlyAssignedGlobalOdom.rotate(currentlyRegisteredPc->sensor_orientation_);

			currentlyRegisteredPc->sensor_origin_ = Eigen::Vector4f(0,0,0,0);
			currentlyRegisteredPc->sensor_orientation_ = Eigen::Quaternionf::Identity();

			Eigen::Affine3f odometryIncrement = lastGlobalOdom.inverse()*currentlyAssignedGlobalOdom;
			std::cout << "odometry increment:\n";
			std::cout<< odometryIncrement.matrix()<<"\n";
			currentInitialFit = lastFit * odometryIncrement ; 
			pcl::transformPointCloud(*currentlyRegisteredPc,*currentlyRegisteredPc,currentInitialFit );
			addMessage("readed pc, press r to register");
		}
		else
		{
			std::cout <<"No more \n";
			
		}
	}
	if (event.getKeySym()=="r" && event.keyUp())
	{
		Eigen::Affine3f tr;
		registration_accepted = false;
		//bool res = registerNDT(metascan, *currentlyRegisteredPc, tr);
		bool res = registerICP(metascan, *currentlyRegisteredPc, tr);
		
		if (res)
		{
			pcl::transformPointCloud(*currentlyRegisteredPc,*currentlyRegisteredPc, tr);
			currentFitCandidate =   tr * currentInitialFit;
			addMessage("registration OK, press a to accept");
		}
		else
		{
			std::cout <<"registration failed\n";
			addMessage("registration NOK, l to load next scan");
		}
	}
	if (event.getKeySym()=="a" && event.keyUp())
	{
		registration_accepted = true;
		metascan += *currentlyRegisteredPc;
		lastGlobalOdom = currentlyAssignedGlobalOdom;
		lastFit = currentFitCandidate;
		addMessage("registration accepted");
		
	}
	p.removeAllPointClouds();
	pcl::visualization::PointCloudColorHandlerCustom<PointT> metascanH(metascan.makeShared(), 0, 0, 255);
	p.addPointCloud<PointT> (metascan.makeShared(),metascanH,"metascan" );
	
	pcl::visualization::PointCloudColorHandlerCustom<PointT> scanH(currentlyRegisteredPc, 255, 0, 0);
	p.addPointCloud<PointT> (currentlyRegisteredPc,scanH, "scan");



}

int main (int argc, char** argv)
{
  myCFG.readConfig("incremental.xml");
  for (int i =0; i < 25; i++)
  {
	  std::stringstream ss;
	  ss<<"text"<<i;
	  p.addText("...",10, i*15, 10,0,1,0, ss.str());
  }
  msg.resize(25);
  currentlyRegisteredPc = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
  // adding to metascan
  std::vector<int> p_file_indices;
  p_file_indices = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
  pcl::PointCloud<PointT> tmp;
  for (int i=0; i < p_file_indices.size(); i++)
  {
	std::cout <<"register :"<< argv[p_file_indices[i]]<<"\n";
	pcNames.push_back(argv[p_file_indices[i]]);
  }
  lastGlobalOdom.setIdentity();
  lastFit.setIdentity();

  pcl::io::loadPCDFile(pcNames[0],tmp);
  
  lastGlobalOdom.translation()<< tmp.sensor_origin_[0],tmp.sensor_origin_[1],tmp.sensor_origin_[2];
  lastGlobalOdom.rotate(tmp.sensor_orientation_);

  pcl::transformPointCloud(tmp,tmp, lastGlobalOdom);
  lastFit = lastGlobalOdom;
  metascan += tmp;
  metascan.sensor_origin_ = Eigen::Vector4f(0,0,0,0);
  metascan.sensor_orientation_ = Eigen::Quaternionf::Identity();

  p.registerKeyboardCallback (keyboardEventOccurred, (void*)&p);
  p.spin();
}
/* ]--- */