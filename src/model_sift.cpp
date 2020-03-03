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


namespace pcl
{
  template<>
    struct SIFTKeypointFieldSelector<PointXYZ>
    {
      inline float
      operator () (const PointXYZ &p) const
      {
    return p.z;
      }
    };
}

ros::Publisher pub, pub_raw;
const float min_scale = 0.2f; //0.01 [pcl::VoxelGrid::applyFilter] Leaf size is too small for the input dataset. Integer indices would overflow.
const int n_octaves = 5;
const int n_scales_per_octave = 3;
const float min_contrast = 0.01f; //I see some paper use 0.5 as threshold


int main(int argc, char** argv){
    ros::init(argc, argv, "pcd_reader");
    ros::NodeHandle nh("~");

    std::string pcd_path = argv[1];
    pub_raw = nh.advertise<sensor_msgs::PointCloud2>("pc",1000);
    pub = nh.advertise<sensor_msgs::PointCloud2>("pc_model",1000);
    // std::string ply_file = "/home/d300/catkin_carol/lidar_frame/PC_315966449519192000.ply";
    std::cout << "Reading " << pcd_path << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
    if(pcl::io::loadPCDFile<pcl::PointXYZ> (pcd_path, *cloud_xyz) == -1) // load the file
    {
      PCL_ERROR ("Couldn't read file");
      return -1;
    }

    std::cout << "Point # : " << cloud_xyz->points.size () <<std::endl;


    pcl::console::TicToc time;
    time.tic();
    // Estimate the sift interest points using z values from xyz as the Intensity variants
    pcl::SIFTKeypoint<pcl::PointXYZ, pcl::PointWithScale> sift;
    pcl::PointCloud<pcl::PointWithScale> result;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ> ());
    sift.setSearchMethod(tree);
    sift.setScales(min_scale, n_octaves, n_scales_per_octave);
    sift.setMinimumContrast(min_contrast);
    sift.setInputCloud(cloud_xyz);
    sift.compute(result);
    std::cout<<"Computing the SIFT points takes "<<time.toc()/1000<<"seconds"<<std::endl;
    std::cout << "No. of SIFT points in the result are " << result.points.size () << std::endl;
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp (new pcl::PointCloud<pcl::PointXYZ>);
    copyPointCloud(result, *cloud_temp);

    sensor_msgs::PointCloud2 raw, out;
    pcl::toROSMsg(*cloud_xyz, raw);
    pcl::toROSMsg(*cloud_temp, out);
    out.header.frame_id = "scan";
    raw.header.frame_id = "scan";

    ros::Rate r(1);
    while(ros::ok()){
        ros::spinOnce();
        pub_raw.publish(raw);
        pub.publish(out);
        r.sleep();
    }
}