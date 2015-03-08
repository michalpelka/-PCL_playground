
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
#include "viewerlog.hpp"
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
dataTransformation inputXML;
dataTransformation outputXML;
 std::vector<std::string> indices;

pcl::PointCloud<PointT> metascan;
std::vector<std::string> pcNames;
pcl::visualization::PCLVisualizer p;
logViewer* logger;
Eigen::Affine3f lastGlobalOdom;
Eigen::Affine3f lastFit;
Eigen::Affine3f currentFitCandidate;
Eigen::Affine3f currentlyAssignedGlobalOdom;
Eigen::Affine3f currentInitialFit;
std::string currentFileName;
  std::string outputXMLFn;

int currentPointcloud =0;
pcl::PointCloud<PointT>::Ptr currentlyRegisteredPc;
bool registration_accepted = false;
configReader myCFG;
std::vector<std::string> msg;

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

void loadNextPc()
{
		currentPointcloud++;
		if (currentPointcloud < indices.size())
		{

			std::cout << "loading pointcloud "<<indices[currentPointcloud]<<"\n";

			inputXML.getPointcloudName(indices[currentPointcloud],currentFileName);	
			currentlyRegisteredPc = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
			pcl::io::loadPCDFile(currentFileName, *currentlyRegisteredPc);
			
			currentlyAssignedGlobalOdom.setIdentity();
			inputXML.getAffine(indices[currentPointcloud],currentlyAssignedGlobalOdom.matrix()),

			currentlyRegisteredPc->sensor_origin_ = Eigen::Vector4f(0,0,0,0);
			currentlyRegisteredPc->sensor_orientation_ = Eigen::Quaternionf::Identity();

			Eigen::Affine3f odometryIncrement = lastGlobalOdom.inverse()*currentlyAssignedGlobalOdom;
			std::cout << "odometry increment:\n";
			std::cout<< odometryIncrement.matrix()<<"\n";
			currentInitialFit = lastFit * odometryIncrement ; 
			pcl::transformPointCloud(*currentlyRegisteredPc,*currentlyRegisteredPc,currentInitialFit );
			logger->addMessage("readed pc, press r to register");
		}
		else
		{
			std::cout <<"No more \n";
			
		}
}
void registerScan()
{
		Eigen::Affine3f tr;
		registration_accepted = false;
		//bool res = registerNDT(metascan, *currentlyRegisteredPc, tr);
		bool res;
		if (myCFG.registrationMethod.compare("ndt")==0)	res= registerNDT(metascan, *currentlyRegisteredPc, tr);
		if (myCFG.registrationMethod.compare("icp")==0)	res= registerICP(metascan, *currentlyRegisteredPc, tr);

		if (res)
		{
			pcl::transformPointCloud(*currentlyRegisteredPc,*currentlyRegisteredPc, tr);
			currentFitCandidate =   tr * currentInitialFit;
			logger->addMessage("registration OK, press a to accept");
		}
		else
		{
			std::cout <<"registration failed\n";
			logger->addMessage("registration NOK, l to load next scan");
		}
}
void accept()
{
		registration_accepted = true;
		metascan += *currentlyRegisteredPc;
		lastGlobalOdom = currentlyAssignedGlobalOdom;
		lastFit = currentFitCandidate;
		logger->addMessage("registration accepted");
		outputXML.setAffine(indices[currentPointcloud], lastFit.matrix());
		outputXML.setPointcloudName(indices[currentPointcloud], currentFileName);
		std::cout <<"saving model to " << outputXMLFn<<"\n";
		outputXML.saveFile(outputXMLFn);
}
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void)
{
	if (event.getKeySym()=="l" && event.keyUp())
	{
		loadNextPc();
	}
	if (event.getKeySym()=="r" && event.keyUp())
	{
		registerScan();
	}
	if (event.getKeySym()=="a" && event.keyUp())
	{
		accept();		
	}
	if (event.getKeySym()=="q"&& event.keyUp())
	{
		loadNextPc();
		registerScan();
		accept();		
	}

	p.removeAllPointClouds();
	pcl::visualization::PointCloudColorHandlerCustom<PointT> metascanH(metascan.makeShared(), 0, 0, 255);
	p.addPointCloud<PointT> (metascan.makeShared(),metascanH,"metascan" );
	
	pcl::visualization::PointCloudColorHandlerCustom<PointT> scanH(currentlyRegisteredPc, 255, 0, 0);
	p.addPointCloud<PointT> (currentlyRegisteredPc,scanH, "scan");
}

int main (int argc, char** argv)
{
   std::cout <<"USAGE:\n";
   std::cout <<argv[0]<<" parameters inputModel.xml outputModel.xml\n";

   if (argc != 4) return -1;
   std::string configurationFile =  argv[1];
   std::string inputXMLFn = argv[2];
   outputXMLFn = argv[3];
  
  myCFG.readConfig(configurationFile);
  logger =new logViewer(&p,25);
  currentlyRegisteredPc = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
  

  inputXML.loadFile(inputXMLFn);
 
  inputXML.getAllScansId(indices);

  pcl::PointCloud<PointT> tmp;
  for (int i=0; i < indices.size(); i++)
  {
	std::cout <<"register :"<< indices[i]<<"\n";
  }
  lastGlobalOdom.setIdentity();
  lastFit.setIdentity();
  std::string fn;
  inputXML.getPointcloudName(indices[0],fn),
  pcl::io::loadPCDFile(fn,tmp);
  
  inputXML.getAffine(indices[0],lastGlobalOdom.matrix()),
  
  pcl::transformPointCloud(tmp,tmp, lastGlobalOdom);
  lastFit = lastGlobalOdom;
  metascan += tmp;
  metascan.sensor_origin_ = Eigen::Vector4f(0,0,0,0);
  metascan.sensor_orientation_ = Eigen::Quaternionf::Identity();

  p.registerKeyboardCallback (keyboardEventOccurred, (void*)&p);

  for (int i=0; i< myCFG.autoCount; i++)
  {
	  loadNextPc();
		registerScan();
		accept();		
  }
  p.spin();
}
/* ]--- */