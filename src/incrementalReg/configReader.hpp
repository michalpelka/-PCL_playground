#ifndef __CFG_READER_HPP
#define __CFG_READER_HPP

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

class configReader
{
public:
	std::string regMethod;
	
	double ndt_step_size;
	float ndt_res;
	int ndt_iter;
	double ndt_trans_eps;
	
	
	float ipc_radius;
	
    int icp_MaximumIterations;
    float icp_CorrespondenceDistance;
    float icp_RANSACOutlierRejectionThreshold;

	void readConfig(std::string cfgfn);
};

#endif