#include "dataTransformations.hpp"
#include <boost/property_tree/detail/xml_parser_writer_settings.hpp>
#include <boost/foreach.hpp>	

	bool dataTransformation::loadFile(std::string fn)
	{
		pt_.clear();
		bool ret = false;
		boost::property_tree::read_xml(fn, pt_);
		ret = true;
		return ret;
	}

	bool dataTransformation::saveFile(std::string fn)
	{
		boost::property_tree::write_xml(fn, pt_, std::locale(), boost::property_tree::xml_writer_make_settings<boost::property_tree::ptree::key_type>(' ', 1u));
		return true;
	}
	
	bool dataTransformation::getAffine (std::string scanId, Eigen::Matrix4f &matrix)
	{
		if (!checkIfExists("Model.Transformations."+scanId+".Affine.Type")) return false; 
		if (!checkIfExists("Model.Transformations."+scanId+".Affine.Data")) return false; 

		std::string type  = pt_.get<std::string>("Model.Transformations."+scanId+".Affine.Type");
		std::string data  = pt_.get<std::string>("Model.Transformations."+scanId+".Affine.Data");
		matrix = Eigen::Matrix4f::Identity();
		if (type.compare("matrix4f")==0)
		{
			std::stringstream ss (data);
			ss>>matrix(0,0)>>matrix(0,1)>>matrix(0,2)>>matrix(0,3);
			ss>>matrix(1,0)>>matrix(1,1)>>matrix(1,2)>>matrix(1,3);
			ss>>matrix(2,0)>>matrix(2,1)>>matrix(2,2)>>matrix(2,3);
			ss>>matrix(3,0)>>matrix(3,1)>>matrix(3,2)>>matrix(3,3);
			return true;
		}
		if (type.compare("Vector3f_Quaternionf")==0)
		{
			Eigen::Quaternionf q;
			Eigen::Vector3f o;
			std::stringstream ss (data);
			ss>>o.x()>>o.y()>>o.z();
			ss>>q.x()>>q.y()>>q.z()>>q.w();
			Eigen::Affine3f affine;
			affine.translate(o);
			affine.rotate(q);
			matrix = affine.matrix();
			return true;
		}

		return false;
		
	}

	bool dataTransformation::getPointcloudName (std::string scanId, std::string &fn)
	{
		if (!checkIfExists("Model.Transformations."+scanId+".cloudname")) return false; 
		fn = pt_.get<std::string>("Model.Transformations."+scanId+".cloudname");
		return true;
	}

	///loads affine transform with given scan ID
	bool dataTransformation::getAffine (std::string scanId, Eigen::Vector3f &origin, Eigen::Quaternionf &quat)
	{
		if (!checkIfExists("Model.Transformations."+scanId+".Affine.Type")) return false; 
		if (!checkIfExists("Model.Transformations."+scanId+".Affine.Data")) return false; 

		
		std::string type  = pt_.get<std::string>("Model.Transformations."+scanId+".Affine.Type");
		std::string data  = pt_.get<std::string>("Model.Transformations."+scanId+".Affine.Data");
		
		if (type.compare("matrix4f")==0)
		{
			Eigen::Matrix4f matrix;
			std::stringstream ss (data);
			ss>>matrix(0,0)>>matrix(0,1)>>matrix(0,2)>>matrix(0,3);
			ss>>matrix(1,0)>>matrix(1,1)>>matrix(1,2)>>matrix(1,3);
			ss>>matrix(2,0)>>matrix(2,1)>>matrix(2,2)>>matrix(2,3);
			ss>>matrix(3,0)>>matrix(3,1)>>matrix(3,2)>>matrix(3,3);
			Eigen::Affine3f ff;
			ff.matrix() = matrix;
			origin = ff.translation();
			quat = ff.rotation();
			return true;
		}
		if (type.compare("Vector3f_Quaternionf")==0)
		{
			
			std::stringstream ss (data);
			ss>>origin.x()>>origin.y()>>origin.z();
			ss>>quat.x()>>quat.y()>>quat.z()>>quat.w();
			return true;
		}

		return false;
		
	}
	


	///// loads affine transform with given scan ID
	//bool dataTransformation::getAffine (std::string scanId, Eigen::Matrix4f &matrix);
	///// loads affine transform with given scan ID
	//bool dataTransformation::getAffine (std::string scanId, Eigen::Vec3f &origin, Eigen::Vec3f &quat);
	///// get pointcloud name of scanId
	//bool dataTransformation::getPointcloudName (std::string scanId, std::string &fn);
	//
	///// retrieve all scans in file 
	//void dataTransformation::getAllScansId(std::vector<std::string> &ids);
	//
	void dataTransformation::setAffine (std::string scanId, Eigen::Matrix4f &matrix)
	{
	

		pt_.put("Model.Transformations."+scanId+".Affine.Type", "matrix4f");
		std::stringstream ss;
		ss << matrix(0,0) << " "<< matrix(0,1) << " "<< matrix(0,2) << " "<< matrix(0,3) << " ";
		ss << matrix(1,0) << " "<< matrix(1,1) << " "<< matrix(1,2) << " "<< matrix(1,3) << " ";
		ss << matrix(2,0) << " "<< matrix(2,1) << " "<< matrix(2,2) << " "<< matrix(2,3) << " ";
		ss << matrix(3,0) << " "<< matrix(3,1) << " "<< matrix(3,2) << " "<< matrix(3,3) << " ";
		pt_.put("Model.Transformations."+scanId+".Affine.Data", ss.str());
		
	}


	void dataTransformation::setAffine (std::string scanId, Eigen::Vector3f origin, Eigen::Quaternionf quat)
	{
		//check if node exists
		addIfNotExists("Model.Transformations."+scanId);

		pt_.put("Model.Transformations."+scanId+".Affine.Type", "Vector3f_Quaternionf");
		std::stringstream ss;
		ss << origin[0] << " "<<origin.x()<< " "<<origin.y()<< " "<<quat.z()<< " "<<quat.x()<< " "<<quat.y()<< " "<<quat.z()<< " "<<quat.w();
		pt_.put("Model.Transformations."+scanId+".Affine.Data", ss.str());
	}
	
	void dataTransformation::addIfNotExists(std::string path)
	{
		boost::optional<boost::property_tree::ptree&> opt_loc_pt_  = pt_.get_child_optional(path);
		
		if (!opt_loc_pt_)
		{
			pt_.add_child(path, boost::property_tree::ptree());
		}
	}
	
	bool dataTransformation::checkIfExists(std::string path)
	{
		boost::optional<boost::property_tree::ptree&> opt_loc_pt_  = pt_.get_child_optional(path);
		return opt_loc_pt_;
	}
	
	void dataTransformation::setPointcloudName (std::string scanId, std::string fn)
	{
		pt_.put("Model.Transformations."+scanId+".cloudname", fn);
	}
	
	void dataTransformation::getAllScansId(std::vector<std::string> &ids)
	{
		ids.clear();
		BOOST_FOREACH(boost::property_tree::ptree::value_type &v, pt_.get_child("Model.Transformations")) {
			ids.push_back(v.first);
		}

	}
	//void dataTransformation::setAffine (std::string scanId, Eigen::Vec3f &origin, Eigen::Vec3f &quat);
	//void dataTransformation::setPointcloudName (std::string scanId, std::string &fn);