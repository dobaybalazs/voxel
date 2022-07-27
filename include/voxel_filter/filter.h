#pragma once

//ROS includes
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include "voxel_filter/filter_nodeConfig.h"

//PCL includes
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>

//STL includes
#include <string>
#include <iostream>

namespace params{
    extern std::string topic_name;
    extern double min_x,min_y,min_z;
    extern double max_x,max_y,max_z;
    extern float lx,ly,lz;
    extern bool useVoxelF;
};


class Filter{
    ros::Subscriber sub;
    ros::Publisher pub;
    public:
        Filter(ros::NodeHandlePtr);

        void voxelFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr,pcl::PointCloud<pcl::PointXYZI>::Ptr);

        void filterCallback(const pcl::PointCloud<pcl::PointXYZI>::Ptr);

};