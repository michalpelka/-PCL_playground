#ifndef __DATA_TRANSF_HPP
#define __DATA_TRANSF_HPP

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/optional/optional.hpp>
#include <Eigen/Eigen>

class dataTransformation
{
public:
	
	/// loads XML file containing set of transformations
	bool loadFile(std::string fn);
	/// saveFileves XML set of transformations
	bool saveFile(std::string fn);
	/// loads affine transform with given scan ID
	
	bool getAffine (std::string scanId, Eigen::Matrix4f &matrix);
	///loads affine transform with given scan ID
	bool getAffine (std::string scanId, Eigen::Vector3f &origin, Eigen::Quaternionf &quat);
	bool getPointcloudName (std::string scanId, std::string &fn);
	

	
	/// retrieve all scans in file 
	void getAllScansId(std::vector<std::string> &ids);
	
	void setAffine (std::string scanId, Eigen::Matrix4f &matrix);
	void setAffine (std::string scanId, Eigen::Vector3f origin, Eigen::Quaternionf quat);
	void setPointcloudName (std::string scanId, std::string fn);
		
private:
	boost::property_tree::ptree pt_;

	void addIfNotExists(std::string path);
	bool checkIfExists(std::string path);
};

#endif