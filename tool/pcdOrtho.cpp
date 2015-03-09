#include <pcl/io/png_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>


//convenient typedefs
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

//convenient structure to handle our pointclouds
struct PCD
{
  PointCloud::Ptr cloud;
  std::string f_name;

  PCD() : cloud (new PointCloud) {};
};

////////////////////////////////////////////////////////////////////////////////
/** \brief Load a set of PCD files that we want to register together
* \param argc the number of arguments (pass from main ())
* \param argv the actual command line arguments (pass from main ())
* \param models the resultant vector of point cloud datasets
*/
void loadData (int argc, char **argv, std::vector<PCD, Eigen::aligned_allocator<PCD> > &models)
{
	std::string extension (".pcd");

	for (int i = 2; i < argc; i++)
	{
		std::string fname = std::string (argv[i]);

		if (fname.size () <= extension.size ())
			continue;

		std::transform (fname.begin (), fname.end (), fname.begin (), (int(*)(int))tolower);

		//check that the argument is a pcd file
		if (fname.compare (fname.size () - extension.size (), extension.size (), extension) == 0)
		{
			// Load the cloud and saves it into the list of models
			PCD m;
			m.f_name = argv[i];
			pcl::io::loadPCDFile (argv[i], *m.cloud);
			//remove NAN points from the cloud
			std::vector<int> indices;
			pcl::removeNaNFromPointCloud(*m.cloud,*m.cloud, indices);

			models.push_back (m);
			PCL_INFO("Added: %s\n",argv[i]);
		}
	}
}


int main (int argc, char** argv)
{
	// Load data
	std::vector<PCD, Eigen::aligned_allocator<PCD> > data;
	loadData (argc, argv, data);

	// Check user input
	if (data.empty ())
	{
		PCL_ERROR ("Syntax is: %s output.png input1.pcd [*]", argv[0]);
		PCL_ERROR ("[*] - multiple files can be added. All files are merged and saved into output.pcd");
		return (-1);
	}
	PCL_INFO ("Loaded %d datasets.", (int)data.size ());


	PointCloud::Ptr result (new PointCloud);
	for(int i=0;i<data.size();++i)
	{
		for(int j=0;j<data[i].cloud->size();++j)
		{
			result->push_back((*data[i].cloud)[j]);
		}
	}
	PCL_INFO("Saving to: %s (size=%d)\n",argv[1],result->size());
	pcl::io::savePNGFile(argv[1],*result);
}