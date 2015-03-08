/* \author Kripasindhu Sarkar */

#include<pcl/visualization/pcl_plotter.h>

#include<iostream>
#include<vector>
#include<utility>
#include<math.h>  //for abs()
#include"dataFramework\dataTransformations.hpp"
using namespace std;
using namespace pcl::visualization;

//............................................................................




void createTrayectory (dataTransformation &tr1, std::vector<double> &x, std::vector<double> &y)
{
	std::vector<std::string> ids;
	tr1.getAllScansId(ids);

	x.resize(ids.size());
	y.resize(ids.size());

	for (int i =0 ; i < ids.size(); i++)
	{
		Eigen::Vector3f f = Eigen::Vector3f(0,0,0);
		Eigen::Affine3f affine;
		
		tr1.getAffine(ids[i],affine.matrix());
		Eigen::Vector3f out = affine * f;
		//std::cout << out.x()<<"\t" << out.y()<<"\n";

		x[i]=static_cast<double>(out.x());
		y[i]=static_cast<double>(out.y());
		
	}
}


void addTrayectory(std::string xmlName, PCLPlotter * p)
{
  dataTransformation tr1;
  tr1.loadFile(xmlName);
  std::vector<double> ax;
  std::vector<double> ay;
  createTrayectory(tr1, ax,ay);

  //addPlotData (std::vector< double > const &array_x, std::vector< double >const &array_y, char const *name="Y Axis", int type=vtkChart::LINE, std::vector< char > const &color=std::vector< char >())
  p->addPlotData(ax,ay,xmlName.c_str());
  
  p->setXRange(-50,50);
  p->setYRange(-50,50);

  
}
int main (int argc, char * argv [])
{
  

  std::cout <<"usage:\n";
  std::cout << argv[0] <<" MODEL1.XML MODEL2.XML ...\n";
  if (argc==1) return -1;
  PCLPlotter *plotter = new PCLPlotter ("My Plotter");
  plotter->setShowLegend (true);
 
  for (int i=1; i < argc ; i++)
  {
	  std::string model = argv[i];
	  std::cout << "loading model " << model << "\n";
	  addTrayectory(model,plotter);
  }
  //defining a plotter


  //setting some properties
	

  


  plotter->spin();

  return 1;
}