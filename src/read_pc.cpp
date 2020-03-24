// STL
#include <iostream>
#include <ros/ros.h>

// PCL
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/pfh.h>
#include <pcl/features/fpfh.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
#include <fstream>
#include <sstream>
#include <string>
#include <signal.h>

ros::Publisher pub;

int main(int argc, char** argv){
    ros::init(argc, argv, "pcd_reader");
    ros::NodeHandle nh("~");

    std::string pcd_path = argv[1];
    pub = nh.advertise<sensor_msgs::PointCloud2>("pc",1000);
    // std::string ply_file = "/home/d300/catkin_carol/lidar_frame/PC_315966449519192000.ply";
    std::cout << "Reading " << pcd_path << std::endl;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZI>);
    if(pcl::io::loadPLYFile<pcl::PointXYZI> (pcd_path, *cloud_xyz) == -1) // load the file
    {
      PCL_ERROR ("Couldn't read file");
      return -1;
    }

    sensor_msgs::PointCloud2 out;
    pcl::toROSMsg(*cloud_xyz, out);
    out.header.frame_id = "scan";

    ros::Rate r(1);
    while(ros::ok()){
        ros::spinOnce();
        pub.publish(out);
        r.sleep();
    }
}