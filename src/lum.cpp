#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/lum.h>
#include <pcl/registration/correspondence_estimation.h>
#include <iostream>
#include <string>
#include <vector>
#include "dataFramework\dataTransformations.hpp"
#include <Eigen\Eigen>

typedef pcl::PointXYZ PointType;



dataTransformation model;
dataTransformation modelAfterLum;

std::vector<std::string> cloudIds;
std::vector<pcl::PointCloud<PointType>::Ptr> clouds;

Eigen::Vector3f getOrigin (int id )
{
	Eigen::Affine3f mm;
	Eigen::Vector3f origin;
	model.getAffine(cloudIds[id], mm.matrix());
	origin = mm * Eigen::Vector3f(0,0,0);
	return origin;
}


int main (int argc, char **argv)
{
	/// loading model
	model.loadFile("modelRegICP.xml");
	///params
	double dist = 2.5;
	pcl::console::parse_argument (argc, argv, "-d", dist);

	int iter = 10;
	pcl::console::parse_argument (argc, argv, "-i", iter);

	int lumIter = 1;
	pcl::console::parse_argument (argc, argv, "-l", lumIter);

	pcl::registration::LUM<PointType> lum;
	lum.setMaxIterations (lumIter);
	lum.setConvergenceThreshold (0.0);



	model.getAllScansId(cloudIds);
	for (size_t i = 0; i < cloudIds.size (); i++)
	{
		std::string fn;
		Eigen::Matrix4f tr;

		model.getPointcloudName(cloudIds[i], fn);
		modelAfterLum.setPointcloudName(cloudIds[i], fn);
		model.getAffine(cloudIds[i], tr);

		pcl::PointCloud<PointType>::Ptr pc (new pcl::PointCloud<PointType>);
		pcl::io::loadPCDFile (fn, *pc);
		pcl::transformPointCloud(*pc, *pc, tr);
		clouds.push_back(pc);
		std::cout << "loading file: " << fn << " size: " << pc->size () << std::endl;
		lum.addPointCloud (clouds[i]);
	}

	for (int i = 0; i < 10; i++)
	{
		for (size_t i = 1; i < clouds.size (); i++)
			for (size_t j = 0; j < i; j++)
			{
				Eigen::Vector4f ci, cj;
				pcl::compute3DCentroid (*(clouds[i]), ci);
				pcl::compute3DCentroid (*(clouds[j]), cj);
				Eigen::Vector4f diff = ci - cj;

				//std::cout << i << " " << j << " " << diff.norm () << std::endl;

				if(diff.norm () < 5.0 && (i - j == 1 || i - j > 20))
				{
					if(i - j > 20)
						std::cout << "add connection between " << i << " (" << cloudIds[i] << ") and " << j << " (" << cloudIds[j] << ")" << std::endl;
					pcl::registration::CorrespondenceEstimation<PointType, PointType> ce;
					ce.setInputTarget (clouds[i]);
					ce.setInputCloud (clouds[j]);
					pcl::CorrespondencesPtr corr (new pcl::Correspondences);
					ce.determineCorrespondences (*corr, 2.5f);
					if (corr->size () > 2)
						lum.setCorrespondences (j, i, corr);
				}
			}


			lum.compute ();
			for(int i = 0; i < lum.getNumVertices (); i++)
			{
				//std::cout << i << ": " << lum.getTransformation (i) (0, 3) << " " << lum.getTransformation (i) (1, 3) << " " << lum.getTransformation (i) (2, 3) << std::endl;
				clouds[i] = lum.getTransformedCloud (i);
			}
	}
	for(size_t i = 0; i < lum.getNumVertices (); i++)
	{
		//std::cout << i << ": " << lum.getTransformation (i) (0, 3) << " " << lum.getTransformation (i) (1, 3) << " " << lum.getTransformation (i) (2, 3) << std::endl;
		Eigen::Affine3f tr2;
		model.getAffine(cloudIds[i], tr2.matrix());
		Eigen::Affine3f tr = lum.getTransformation(i);

		Eigen::Affine3f final = tr * tr2;
		modelAfterLum.setAffine(cloudIds[i], final.matrix());
	}
	modelAfterLum.saveFile("modelLUM.xml");


	return 0;
}