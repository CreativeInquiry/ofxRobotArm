#include "FileLoader.h"
using namespace ofxRobotArm;
URDFLoader::URDFLoader(){

}

URDFLoader::~URDFLoader(){

}

void URDFLoader::load(string path){
    yaml.load(path);
}

INFOLoader::INFOLoader(){

}
INFOLoader::~INFOLoader(){

}

void INFOLoader::load(string path){
    
    yaml.load(path);

    yaml["urdf_file_name"];
    yaml["starting_config"];
    yaml["starting_config"];
    yaml["joint_limits"];
    yaml["displacements"];
    yaml["velocity_limits"];
    yaml["rot_offsets"];
    yaml["axis_types"];

    
}