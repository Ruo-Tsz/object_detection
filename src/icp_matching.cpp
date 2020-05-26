#include <iostream>
#include <ros/ros.h>

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

#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_features.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>

#include <pcl/kdtree/kdtree_flann.h>

using namespace std;

string scene;

int max_iteration = 50;
double max_corresDist = 5.0f; //0.04 seems to in local optimal, converge to initial sac result (4 cm tight threshold)  >0.4 good太小無法配準
double EucFitnessEps = 0.05f; //score ->所有點距離均方差 0.2->0.05好一些 ->0.01跟0.05差不多
double TransEps = 1e-10f;

template<class T>
string ConvertToString(T value){
  stringstream ss;
  ss << value;
  return ss.str();
}

Eigen::Matrix4f do_icp(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_source, pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_target, Eigen::Matrix4f &initial_guess){
    pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
    pcl::PointCloud <pcl::PointXYZI>::Ptr icp_result (new pcl::PointCloud <pcl::PointXYZI> );
    pcl::console::TicToc timer_icp;
    timer_icp.tic();
    icp.setInputSource(cloud_source);
    icp.setInputTarget(cloud_target);
    icp.setMaximumIterations(max_iteration);
    icp.setMaxCorrespondenceDistance(max_corresDist);
    icp.setEuclideanFitnessEpsilon(EucFitnessEps);
    icp.setTransformationEpsilon(TransEps);
    icp.align(*icp_result, initial_guess);
    cout <<"----------------------\nFor ICP:";
    cout <<"has converged: "<< icp.hasConverged() <<",score: "<<icp.getFitnessScore()<< endl;
    cout << icp.getConvergeCriteria()<<endl;
    
    cout<<"ICP Tranformation is\n "<<icp.getFinalTransformation()<<endl;
    double icp_times_up = timer_icp.toc()/1000.0;
    string icp_time = ConvertToString(icp_times_up);
    string icp_score = ConvertToString(icp.getFitnessScore());
    cout <<"Need "<<icp_times_up<<" secs.\n";
    cout<< "The final size: "<<icp_result->points.size()<<endl;

    Eigen::Matrix4f icp_trans = icp.getFinalTransformation();
    return icp_trans;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "test_icp");
    ros::NodeHandle nh("~");
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("icp_result", 1000);
    ros::Publisher pub_model = nh.advertise<sensor_msgs::PointCloud2>("model", 1000);
    
    std::string scene_path = "/home/d300/catkin_carol/src/object_detection/segmented_car_from_scene/";
    std::string model_path = "/home/d300/catkin_carol/src/object_detection/model/3D_Models/Audi_R8/scan/temp/";
    std::string model_file, model, scene_file, scene;
    nh.getParam("model",model);
    nh.getParam("scene",scene);
    model_file = model_path + model;
    scene_file = scene_path + scene;

    std::cout << "Reading " << scene_file << std::endl;
    std::cout << "Reading " << model_file << std::endl;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_source (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_target (new pcl::PointCloud<pcl::PointXYZI>);
    if(pcl::io::loadPCDFile<pcl::PointXYZI> (model_file, *cloud_target) == -1) // load the file
    {
      PCL_ERROR ("Couldn't read file ",model_file);
      return -1;
    }
    if(pcl::io::loadPCDFile<pcl::PointXYZI> (scene_file, *cloud_source) == -1) // load the file
    {
      PCL_ERROR ("Couldn't read file ",scene_file);
      return -1;
    }

    Eigen::Matrix4f initial_guess = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f icp_trans = do_icp(cloud_source, cloud_target, initial_guess);

    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_source (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr original_source (new pcl::PointCloud<pcl::PointXYZI>);
    sensor_msgs::PointCloud2 msg, msg_model;
    pcl::transformPointCloud(*cloud_source, *transformed_source, icp_trans);

    pcl::toROSMsg(*transformed_source, msg);
    pcl::toROSMsg(*cloud_target, msg_model);
    msg.header.frame_id="scan";
    msg.header.stamp = ros::Time::now();
    msg_model.header.frame_id="scan";
    msg_model.header.stamp = ros::Time::now();

    ros::Rate r(10);
    while(ros::ok()){
        pub.publish(msg);
        pub_model.publish(msg_model);
        ros::spinOnce();
        r.sleep();
    }

}