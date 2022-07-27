#include "voxel_filter/filter.h"


Filter::Filter(ros::NodeHandlePtr nh){
    sub = nh->subscribe(params::topic_name,1,&Filter::filterCallback,this);

    pub = nh->advertise<pcl::PointCloud<pcl::PointXYZI>>("filtered_cloud",1);
}

void Filter::voxelFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud,pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud){
    pcl::VoxelGrid<pcl::PointXYZI> grid;
    grid.setInputCloud(input_cloud);
    grid.setLeafSize(params::lx,params::ly,params::lz);
    grid.filter(*output_cloud);
}

void Filter::filterCallback(const pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud){
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    
    std::vector<int> indicies;
    pcl::CropBox<pcl::PointXYZI> roi(true);

    Eigen::Vector4f min(params::min_x,params::min_y,params::min_z,0.0);
    Eigen::Vector4f max(params::max_x,params::max_y,params::max_z,0.0);

    roi.setMin(min);
    roi.setMax(max);
    roi.setInputCloud(input_cloud);
    roi.filter(indicies);

    for(const auto& idx:indicies)
        filtered_cloud->push_back(input_cloud->points[idx]);
    if(params::useVoxelF){
        pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        Filter::voxelFilter(filtered_cloud,output_cloud);
        output_cloud->header = input_cloud->header;
        pub.publish(output_cloud);
        return;
    }
    filtered_cloud->header = input_cloud->header;
    pub.publish(filtered_cloud);
}