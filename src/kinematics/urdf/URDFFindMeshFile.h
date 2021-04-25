
#ifndef URDF_FIND_MESH_FILE_H
#define URDF_FIND_MESH_FILE_H


#include <string>
#include <list>

#include "URDFParser.h"

static bool UrdfFindMeshFile(const std::string& urdf_path, std::string fn,
	const std::string& error_message_prefix,
	std::string* out_found_filename, int* out_type)
{
	if (fn.size() <= 4)
	{
		ofLogWarning()<<("%s: invalid mesh filename '%s'\n", error_message_prefix.c_str(), fn.c_str());
		return false;
	}

	std::string ext;
	std::string ext_ = fn.substr(fn.size() - 4);
	for (std::string::iterator i = ext_.begin(); i != ext_.end(); ++i)
	{
		ext += char(tolower(*i));
	}

	if (ext == ".dae")
	{
		*out_type = UrdfGeometry::FILE_COLLADA;
	}
	else if (ext == ".stl")
	{
		*out_type = UrdfGeometry::FILE_STL;
	}
	else if (ext == ".obj")
	{
		*out_type = UrdfGeometry::FILE_OBJ;
	}
	else if (ext == ".cdf")
	{
		*out_type = UrdfGeometry::FILE_CDF;
	}
	else if (ext == ".vtk")
	{
		*out_type = UrdfGeometry::FILE_VTK;
	}
	else
	{
		ofLogWarning()<<("%s: invalid mesh filename extension '%s'\n", error_message_prefix.c_str(), ext.c_str());
		return false;
	}

	std::string drop_it_file = "file://";
	std::string drop_it_pack = "package://";
	std::string drop_it_model = "model://";
	if (fn.substr(0, drop_it_file.length()) == drop_it_file)
		fn = fn.substr(drop_it_file.length());
	if (fn.substr(0, drop_it_pack.length()) == drop_it_pack)
		fn = fn.substr(drop_it_pack.length());
	else if (fn.substr(0, drop_it_model.length()) == drop_it_model)
		fn = fn.substr(drop_it_model.length());

	std::list<std::string> shorter;
	shorter.push_back("../../");
	shorter.push_back("../");
	shorter.push_back("./");
	int cnt = urdf_path.size();
	for (int i = 0; i < cnt; ++i)
	{
		if (urdf_path[i] == '/' || urdf_path[i] == '\\')
		{
			shorter.push_back(urdf_path.substr(0, i) + "/");
		}
	}
	shorter.push_back("");  // no prefix
	shorter.reverse();

	std::string existing_file;

	for (std::list<std::string>::iterator x = shorter.begin(); x != shorter.end(); ++x)
	{
		std::string attempt = *x + fn;
		ofFile file;
		if(!file.open(attempt)){
			continue;
		}
		existing_file = attempt;
		break;
	}

	if (existing_file.empty())
	{
		ofLogWarning()<<("%s: cannot find '%s' in any directory in urdf path\n", error_message_prefix.c_str(), fn.c_str());
		return false;
	}
	else
	{
		*out_found_filename = existing_file;
		return true;
	}
}

#endif //URDF_FIND_MESH_FILE_H
