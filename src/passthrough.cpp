#include <ros/ros.h>
#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/io/png_io.h>
#include <ctime>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/time.h> // 計時

#include <thread>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;

int main(int argc, char** argv){
    pcl::StopWatch time; // 計時開始
    ros::init(argc, argv, "pass_through");
    ros::NodeHandle nh("~");

    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("filtered",1000);
    ros::Publisher pub_raw = nh.advertise<sensor_msgs::PointCloud2>("original", 1000);
    ros::Publisher pub_z = nh.advertise<sensor_msgs::PointCloud2>("z", 1000);
    ros::Publisher pub_zx = nh.advertise<sensor_msgs::PointCloud2>("z_x", 1000);
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);

    string pcd_path = argv[1];
    
    if(pcl::io::loadPLYFile<pcl::PointXYZI> (pcd_path, *cloud) == -1) // load the file
    {
      PCL_ERROR ("Couldn't read file");
      return -1;
    }

    pcl::PassThrough<pcl::PointXYZI> pass_z;
	pass_z.setInputCloud(cloud);
	pass_z.setFilterFieldName("z");
	pass_z.setFilterLimits(-0.5, -0.2);
	// pass.setFilterFieldName("intensity");
	// pass.setFilterLimits(0.0, 15.0);
	pass_z.setFilterLimitsNegative(true); 
	// pass.setFilterLimitsNegative(false);
	pass_z.filter(*cloud_filtered);

    sensor_msgs::PointCloud2 msg_z;
    pcl::toROSMsg(*cloud_filtered, msg_z);
    msg_z.header.frame_id = "scan";

    pcl::PassThrough<pcl::PointXYZI> pass_x;
	pass_x.setInputCloud(cloud_filtered);
	pass_x.setFilterFieldName("x");
	pass_x.setFilterLimits(-8.5, -4.5);
	pass_x.setFilterLimitsNegative(false); 
    pass_x.filter(*cloud_filtered);

    sensor_msgs::PointCloud2 msg_x;
    pcl::toROSMsg(*cloud_filtered, msg_x);
    msg_x.header.frame_id = "scan";



    pcl::PassThrough<pcl::PointXYZI> pass_y;
	pass_y.setInputCloud(cloud_filtered);
	pass_y.setFilterFieldName("y");
	pass_y.setFilterLimits(-4.5, -2.0);
	pass_y.setFilterLimitsNegative(false); 
    pass_y.filter(*cloud_filtered);

    sensor_msgs::PointCloud2 raw_msg, msg;
    pcl::toROSMsg(*cloud_filtered, msg);
    pcl::toROSMsg(*cloud, raw_msg);
    msg.header.frame_id = "scan";
    raw_msg.header.frame_id = "scan";

    pcl::io::savePCDFile("right_back.pcd", *cloud_filtered, false);

    ros::Rate r(2);
    while(ros::ok()){
        ros::spinOnce();
        pub.publish(msg);
        pub_raw.publish(raw_msg);
        pub_z.publish(msg_z);
        pub_zx.publish(msg_x);
        r.sleep();
    }

    
}