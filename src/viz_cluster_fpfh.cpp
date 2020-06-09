#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>

// PCL
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/pfh.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/shot.h>
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
#include <cmath>
#include <unistd.h>

#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_features.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <signal.h>

ros::Publisher scene_pub, keyPt_pub;
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_source (new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_keyPt (new pcl::PointCloud<pcl::PointXYZI>);
std::ifstream inputFile;
std::vector<int> labels;
using namespace std;

pcl::PointCloud<pcl::PointXYZI>::Ptr read_keyPt(){
    std::string keyPt_path = "/home/d300/catkin_carol/src/object_detection/output/FPFH_whole_scenes_cluster/cluster_pos.csv";
    std::string line;
    inputFile.open(keyPt_pub);
    int counter = 0;
    bool get_pt = false; // get 3 coor for 1 pt
       
    while (getline( inputFile, line,'\n'))  //讀檔讀到跳行字元
	{
	  istringstream templine(line); // string 轉換成 stream
	  string data;
	  while (getline( templine, data,',')) //讀檔讀到逗號
	  {
          if (counter == 0){
            labels.push_back(atoi(data.c_str())); //string 轉換成數字
            counter++;
          }
          else if (counter == 1){
            
          }  
	  }
	}
  inputFile.close();
    return 
}

void signalHandler(int sig){
    ROS_INFO("Shutting down!");
    ros::shutdown();
}

int main(int argc, char** argv){
    ros::init(argc, argv, "viz_cluster");
    ros::NodeHandle nh("~");

    std::string pkg_path = ros::package::getPath("object_detection");
    std::string scene_path = pkg_path + "/segmented_car_from_scene/PC_315966449519192000.pcd";
    // std::string model_path = pkg_path + "/model/3D_Models/Audi_R8/scan/temp/whole_r8";

    scene_pub = nh.advertise<sensor_msgs::PointCloud2>("scene_pc", 1);
    keyPt_pub = nh.advertise<sensor_msgs::PointCloud2>("keyPt_pc", 1);

    signal(SIGINT, signalHandler);

    if(pcl::io::loadPCDFile<pcl::PointXYZ> (scene_path, *cloud_source) == -1){
        PCL_ERROR ("Couldn't read file ",scene_path);
        return -1;
    }

    cloud_keyPt = read_keyPt();

    

}