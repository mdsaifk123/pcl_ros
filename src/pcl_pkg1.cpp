 #include <iostream>
#include <string>
 #include <thread>
 #include <vector>
 #include <pcl/point_types.h>
 #include <pcl/filters/passthrough.h>
 #include <pcl/filters/voxel_grid.h>
 #include <ros/ros.h>
 #include <sensor_msgs/PointCloud2.h>
 #include <pcl_conversions/pcl_conversions.h>
 #include <pcl/PCLPointCloud2.h>
 #include <pcl/filters/filter_indices.h> 
 #include <pcl/segmentation/region_growing_rgb.h>
 
 ros::Publisher pub;
 float distancethreshold;
 void cloud_call_back(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg)
 {
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr raw_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr transform_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
   pcl::fromROSMsg(*cloud_msg, *raw_cloud);

  /* pcl::VoxelGrid<pcl::PointXYZ> sor;
   sor.setInputCloud (raw_cloud);
   sor.setLeafSize (0.05f, 0.05f, 0.05f);
   sor.filter (*transform_cloud); */

   pcl::IndicesPtr indices (new std::vector <int>);
   pcl::removeNaNFromPointCloud (*raw_cloud, *indices);  
   pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
   reg.setInputCloud (raw_cloud);
   reg.setIndices (indices);
   reg.setDistanceThreshold (distancethreshold);
   reg.setPointColorThreshold (6);
   reg.setRegionColorThreshold (5);
   reg.setMinClusterSize (600);

   std::vector <pcl::PointIndices> clusters;
   reg.extract (clusters);
   pcl::PointCloud <pcl::PointXYZRGB>::Ptr coloured_cloud = reg.getColoredCloud ();


   sensor_msgs::PointCloud2 output_msg;
   pcl::toROSMsg(*coloured_cloud, output_msg);

   output_msg.header.frame_id = cloud_msg->header.frame_id;
   output_msg.header.stamp = cloud_msg->header.stamp;

   pub.publish(output_msg);
 }

 

 int main(int argc, char** argv)
 {
    ros::init(argc, argv, "pcl_pkg");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points",1,cloud_call_back);
    pub = nh.advertise<sensor_msgs::PointCloud2>("output",1);
    //ros::spin();
  
    ros::Rate loop_rate(10);
    while(ros::ok())
    {
    nh.param<float>("distancethreshold", distancethreshold , 30);
    ros::spinOnce();
    loop_rate.sleep();
    }
    //return(0);
 }