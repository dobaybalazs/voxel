#include "voxel_filter/filter.h"

std::string params::topic_name = "/left_os1/os1_cloud_node/points";
bool params::useVoxelF = false;
double params::min_x = -4.0;
double params::min_y = -12.0;
double params::min_z = -1.8;
double params::max_x = 100;
double params::max_y = 12.0;
double params::max_z = -0.8;
float params::lx = 0.2;
float params::ly = 0.2;
float params::lz = 0.2;

void setParams(voxel_filter::filter_nodeConfig &config,uint32_t level){
    params::topic_name = config.input_cloud;
    params::useVoxelF = config.useVoxelF;
    params::min_x = config.min_x;
    params::min_y = config.min_y;
    params::min_z = config.min_z;
    params::max_x = config.max_x;
    params::max_y = config.max_y;
    params::max_z = config.max_z;
    params::lx = config.lx;
    params::ly = config.ly;
    params::lz = config.lz;
}

int main(int argc,char** argv){
    ros::init(argc,argv,"filter_node");

    ros::NodeHandlePtr nh=boost::make_shared<ros::NodeHandle>();

    dynamic_reconfigure::Server<voxel_filter::filter_nodeConfig> server;
    dynamic_reconfigure::Server<voxel_filter::filter_nodeConfig>::CallbackType f;

    f = boost::bind(&setParams,_1,_2);
    server.setCallback(f); 

    Filter filter(nh);

    ros::spin();
}