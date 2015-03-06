#include "configReader.hpp""
#include <iostream>

void configReader::readConfig(std::string cfgfn)
{
	using boost::property_tree::ptree;
	ptree pt;
	try {
		read_xml(cfgfn,pt);
	}
	catch(std::exception const&  e)
	{
		std::cerr<< e.what();
		exit(-1);
	}

    try {
		ndt_step_size = pt.get<float>("registration.ndt.step_size");
		ndt_res = pt.get<float>("registration.ndt.res");
		ndt_iter = pt.get<int>("registration.ndt.iterations");
		ndt_trans_eps = pt.get<double>("registration.ndt.eps");;
	
		icp_MaximumIterations = pt.get<int>("registration.icp.MaximumIterations"); 
		icp_CorrespondenceDistance = pt.get<float>("registration.icp.CorrespondenceDistance"); 
		icp_RANSACOutlierRejectionThreshold = pt.get<float>("registration.icp.RANSACOutlierRejectionThreshold"); 

	}
    catch (...)
    {

    }
	    
	std::cout << "ndt_step_size "<<ndt_step_size <<"\n";
	std::cout << "ndt_res "<<ndt_res <<"\n";
	std::cout << "ndt_iter "<<ndt_iter <<"\n";
	std::cout << "ndt_trans_eps "<<ndt_trans_eps <<"\n";

	std::cout << "icp_MaximumIterations "<<icp_MaximumIterations <<"\n";
	std::cout << "icp_CorrespondenceDistance "<<icp_CorrespondenceDistance <<"\n";
	std::cout << "icp_RANSACOutlierRejectionThreshold "<<icp_RANSACOutlierRejectionThreshold <<"\n";

    
}