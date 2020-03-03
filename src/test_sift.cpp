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

/* This examples shows how to estimate the SIFT points based on the 
 * z gradient of the 3D points than using the Intensity gradient as
 * usually used for SIFT keypoint estimation.
 */
#define iteration 50
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
using namespace std;
std::string single;

ros::Publisher pub, raw_pub, pfh_pub, pub_2;
ros::Subscriber sub;
sensor_msgs::PointCloud2 raw, out, pfh_cloud;
// float min_scale = 0.01f; //min scale(sigma) of Gaussian blurring of DoG
// int n_octaves = 6;//the number of octaves (i.e. doublings of scale) to compute
// int n_scales_per_octave = 4;//the number of scales to compute within each octave
// float min_contrast = 0.01f; //I see some paper use 0.5 as threshold
float min_scale = 0.2f; //0.01 [pcl::VoxelGrid::applyFilter] Leaf size is too small for the input dataset. Integer indices would overflow.
int n_octaves = 5;
int n_scales_per_octave = 3;
float min_contrast = 0.01f; //I see some paper use 0.5 as threshold

ofstream outputFile;
string filename_dir = "/home/d300/catkin_carol/src/object_detection/output";
string method;


pcl::PointCloud<pcl::PointXYZ>::Ptr filter_ground(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in){
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.02);//0.01 for bag1

    pcl::ExtractIndices<pcl::PointXYZ> extract;

    int i = 0;
    for(i=0;i<iteration;i++){
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (cloud_in);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
        break;
        }

        // Extract the inliers
        extract.setInputCloud (cloud_in);
        extract.setIndices (inliers);

        extract.setNegative (true);
        extract.filter (*cloud_f);
        cloud_in.swap (cloud_f);
        //cout<<i<<endl;
  }
    return cloud_in;
}






pcl::PointCloud<pcl::PointXYZ>::Ptr crop(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_clusters){
  Eigen::Vector4f box_min,box_max;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_box(new pcl::PointCloud<pcl::PointXYZ> );
  box_min << -30,-30,-30,1;
  box_max << 30,30,30,1;//choose 60x60x60 cube (mask)
  pcl::CropBox<pcl::PointXYZ> in_box;

  in_box.setInputCloud(cloud_clusters);
  in_box.setMin(box_min);
  in_box.setMax(box_max);
  in_box.filter(*cluster_box);

  return cluster_box;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Compute_FPFH(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_raw, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, int k_search){
    // Object for storing the normals.
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_src(new pcl::PointCloud<pcl::FPFHSignature33>());


    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
    normalEstimation.setInputCloud(cloud_raw);
    // 若用半徑,則要夠大 (dataset_dependent),否則可能出現normal nan  導致後面算不出來
    // normalEstimation.setRadiusSearch(0.03);
    normalEstimation.setKSearch(k_search);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
    normalEstimation.setSearchMethod(kdtree);
    normalEstimation.compute(*normals);


    pcl::console::TicToc fpfh_time;
    fpfh_time.tic();
    pcl::FPFHEstimation<pcl::PointXYZ,pcl::Normal,pcl::FPFHSignature33> fpfh;

    fpfh.setInputCloud(cloud_in);
    fpfh.setSearchSurface(cloud_raw);
    fpfh.setKSearch(k_search);
    fpfh.setInputNormals(normals);
    fpfh.setSearchMethod(kdtree);
    
    // fpfh_src.setRadiusSearch(0.05);
    fpfh.compute(*fpfhs_src);
    std::cout<<"Computing the FPFH points takes "<<fpfh_time.toc()/1000<<"seconds"<<std::endl;
    std::cout << "The points # after FPFH: "<< fpfhs_src->points.size()<< std::endl;
    for (int i=0; i<33; i++)
      std::cout << fixed << setprecision(4) << fpfhs_src->points[4].histogram[i] << std::endl;


    // output descriptor
    stringstream kk;
    kk << k_search;
    string filename = filename_dir + "/FPFH_k=" + kk.str() + ".csv";
    outputFile.open(filename);
    for (int k=0; k<fpfhs_src->points.size(); k++){
      outputFile << cloud_raw->header.stamp ;
      for(int j=0; j<33; j++){
        outputFile << "," << fpfhs_src->points[k].histogram[j] ;
      }
      outputFile << endl;
    }
    cout << "Done recording frame " << cloud_raw->header.stamp << endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*fpfhs_src, *cloud_out);
    return cloud_out;
}



pcl::PointCloud<pcl::PointXYZ>::Ptr Compute_PFH(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_raw, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, int k_search){
    // Object for storing the normals.
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    // Object for storing the PFH descriptors for each point.
    pcl::PointCloud<pcl::PFHSignature125>::Ptr descriptors(new pcl::PointCloud<pcl::PFHSignature125>());

    // Estimate the normals.
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
    normalEstimation.setInputCloud(cloud_raw);
    // 若用半徑,則要夠大 (dataset_dependent),否則可能出現normal nan  導致後面算不出來
    // normalEstimation.setRadiusSearch(0.03);
    normalEstimation.setKSearch(k_search);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
    normalEstimation.setSearchMethod(kdtree);
    normalEstimation.compute(*normals);

    cout << normals->points.size() << endl;
    cout << normals->points[1].normal_x << endl;

    for (int i = 0; i < normals->points.size(); i++){
      if (!pcl::isFinite<pcl::Normal>(normals->points[i])){
        PCL_WARN("normals[%d] is not finite\n", i);
      }
    }
 


    pcl::console::TicToc pfh_time;
    pfh_time.tic();
    // PFH estimation object.
    pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh;
    // Only calculate feature at keypoints
    pfh.setInputCloud(cloud_in);
    // Use whole raw data to construct geometrical feature model(including non-keypoint)
    pfh.setSearchSurface(cloud_raw);
    pfh.setKSearch(k_search);
    pfh.setInputNormals(normals);
    pfh.setSearchMethod(kdtree);
    // Search radius, to look for neighbors. Note: the value given here has to be
    // larger than the radius used to estimate the normals.
    // pfh.setRadiusSearch(0.05);


    pfh.compute(*descriptors);
    std::cout<<"Computing the PFH points takes "<<pfh_time.toc()/1000<<"seconds"<<std::endl;
    std::cout << "The points # after PFH: "<< descriptors->points.size()<< std::endl;
    for (int i=0; i<125; i++)
      std::cout << fixed << setprecision(4) << descriptors->points[4].histogram[i] << std::endl;


    // output descriptor
    stringstream kk;
    kk << k_search;
    string filename = filename_dir + "/PFH_k=" + kk.str() + ".csv";
    outputFile.open(filename);
    for (int k=0; k<descriptors->points.size(); k++){
      outputFile << cloud_raw->header.stamp ;
      for(int j=0; j<125; j++){
        outputFile << "," << descriptors->points[k].histogram[j] ;
      }
      outputFile << endl;
    }
    cout << "Done recording frame " << cloud_raw->header.stamp << endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*descriptors, *cloud_out);
    return cloud_out;

}



void callback(const sensor_msgs::PointCloud2 &msg){
    cout << "Get new at " << msg.header.stamp << std::endl;
    std::cout << "Original points: " << msg.width*msg.height <<std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_msg (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(msg, *cloud_msg);

    cloud_in = filter_ground(cloud_msg);
    std::cout<<"After remove ground: " << cloud_msg->points.size() << std::endl;

    // cloud_in = crop(cloud_msg);

    pcl::console::TicToc time;
    time.tic();
    // Estimate the sift interest points using z values from xyz as the Intensity variants
    pcl::SIFTKeypoint<pcl::PointXYZ, pcl::PointWithScale> sift;
    pcl::PointCloud<pcl::PointWithScale> result;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ> ());
    sift.setSearchMethod(tree);
    sift.setScales(min_scale, n_octaves, n_scales_per_octave);
    sift.setMinimumContrast(min_contrast);
    sift.setInputCloud(cloud_in);
    sift.compute(result);
    std::cout<<"Computing the SIFT points takes "<<time.toc()/1000<<"seconds"<<std::endl;
    std::cout << "No. of SIFT points in the result are " << result.points.size () << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp (new pcl::PointCloud<pcl::PointXYZ>);
    copyPointCloud(result, *cloud_temp);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_des (new pcl::PointCloud<pcl::PointXYZ>);
    cloud_des = Compute_PFH(cloud_in,cloud_temp, 20);

    pcl::toROSMsg(*cloud_temp, out);
    pcl::toROSMsg(*cloud_in, raw);
    pcl::toROSMsg(*cloud_des, pfh_cloud);
    out.header.frame_id = "scan";
    raw.header.frame_id = "scan";
    pfh_cloud.header.frame_id = "scan";

    raw_pub.publish(raw);
    pub.publish(out);
    pfh_pub.publish(pfh_cloud);


}


void MySigintHandler(int sig)
{
	//这里主要进行退出前的数据保存、内存清理、告知其他节点等工作
	ROS_INFO("shutting down!");
  outputFile.close();
	ros::shutdown();
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "Sift");
    ros::NodeHandle nh("~");
    pub = nh.advertise<sensor_msgs::PointCloud2>("sift_point",1000);
    raw_pub = nh.advertise<sensor_msgs::PointCloud2>("raw_point",1000);
    pfh_pub = nh.advertise<sensor_msgs::PointCloud2>("pfh_feature", 1000);
    pub_2 = nh.advertise<sensor_msgs::PointCloud2>("sift_compare", 1000);
    
    sub = nh.subscribe("/scan",1,&callback);

    signal(SIGINT, MySigintHandler);

    // string k_search;
    nh.getParam("method", method);
    // nh.getParam("k",k_search);
    // cout << k_search << endl;
    // int k_search_no = std::strtol(k_search.c_str(),nullptr,10);
    int k_search_no = 20;

   

  //////////////////////////////single frame
    std::string ply_file = "/home/d300/catkin_carol/lidar_frame/PC_315966449519192000.ply";
    std::cout << "Reading " << ply_file << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
    if(pcl::io::loadPLYFile<pcl::PointXYZ> (ply_file, *cloud_xyz) == -1) // load the file
    {
      PCL_ERROR ("Couldn't read file");
      return -1;
    }

    std::cout << cloud_xyz->sensor_origin_[0] <<","<< cloud_xyz->sensor_origin_[1] <<","<<cloud_xyz->sensor_origin_[2] << endl;
    // std::cout << Eigen::Affine3f(cloud_xyz->sensor_orientation_) << endl;
    std::cout << "Point # : " << cloud_xyz->points.size () <<std::endl;
    cloud_xyz = filter_ground(cloud_xyz);
    std::cout<<"After remove ground: " << cloud_xyz->points.size() << std::endl;
    
    // Parameters for sift computation

    float min_scale_2 = min_scale;
    int n_octaves_2 = n_octaves;//the number of octaves (i.e. doublings of scale) to compute
    int n_scales_per_octave_2 = n_scales_per_octave;//the number of scales to compute within each octave
    float min_contrast_2 = min_contrast;
   //-----------setting parameters of sift----------------//
   
    pcl::console::parse(argc, argv, "-min_scale", min_scale_2);
    pcl::console::parse(argc, argv, "-n_octaves", n_octaves_2);
    pcl::console::parse(argc, argv, "-n_scales_per_octave", n_scales_per_octave_2);
    pcl::console::parse(argc, argv, "-min_contrast", min_contrast_2);
    pcl::console::parse(argc, argv, "-k", k_search_no);

    std::cout<<"-----------------------------------------\n";
    std::cout<<"min_scale: " << min_scale << std::endl;
    std::cout<<"n_octaves: " << n_octaves << std::endl;
    std::cout<<"n_scales_per_octave: " << n_scales_per_octave << std::endl;
    std::cout<<"min_contrast: " << min_contrast << std::endl;

    

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


    // Copying the pointwithscale to pointxyz so as visualize the cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp (new pcl::PointCloud<pcl::PointXYZ>);
    copyPointCloud(result, *cloud_temp);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_des (new pcl::PointCloud<pcl::PointXYZ>);
    if ( !(method.compare("pfh")) )
      cloud_des = Compute_PFH(cloud_xyz,cloud_temp, k_search_no);
    else if( !(method.compare("fpfh")) )
      cloud_des = Compute_FPFH(cloud_xyz,cloud_temp, k_search_no);
    else
      cout << "Just do sift.\n";
    std::cout<<"-----------------------------------------\n";

    //2
    std::cout<<"min_scale_2: " << min_scale_2 << std::endl;
    std::cout<<"n_octaves_2: " << n_octaves_2 << std::endl;
    std::cout<<"n_scales_per_octave_2: " << n_scales_per_octave_2 << std::endl;
    std::cout<<"min_contrast_2: " << min_contrast_2 << std::endl;
    pcl::console::TicToc time_2;
    time_2.tic();
    // Estimate the sift interest points using z values from xyz as the Intensity variants
    pcl::SIFTKeypoint<pcl::PointXYZ, pcl::PointWithScale> sift_2;
    pcl::PointCloud<pcl::PointWithScale> result_2;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_2(new pcl::search::KdTree<pcl::PointXYZ> ());
    sift.setSearchMethod(tree_2);
    sift.setScales(min_scale_2, n_octaves_2, n_scales_per_octave_2);
    sift.setMinimumContrast(min_contrast_2);
    sift.setInputCloud(cloud_xyz);
    sift.compute(result_2);
    std::cout<<"Computing the SIFT_2 points takes "<<time_2.toc()/1000<<"seconds"<<std::endl;
    std::cout << "No. of SIFT_2 points in the result are " << result_2.points.size () << std::endl;


    // Copying the pointwithscale to pointxyz so as visualize the cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp_2 (new pcl::PointCloud<pcl::PointXYZ>);
    copyPointCloud(result_2, *cloud_temp_2);
    
    sensor_msgs::PointCloud2 out_2;
    pcl::toROSMsg(*cloud_temp_2, out_2);
    out_2.header.frame_id = "scan";
    //2


    sensor_msgs::PointCloud2 raw, out;
    pcl::toROSMsg(*cloud_temp, out);
    pcl::toROSMsg(*cloud_xyz, raw);
    out.header.frame_id = "scan";
    raw.header.frame_id = "scan";
    
    //////////////////////////////single frame
  
    ros::Rate r(2);
    while(ros::ok()){
        ros::spinOnce();
        pub.publish(out);
        raw_pub.publish(raw);
        pub_2.publish(out_2);
        r.sleep();
    }
    
      

    return 0;
  
}