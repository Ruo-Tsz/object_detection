// STL
#include <iostream>
#include <ros/ros.h>

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

#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_features.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>

#include <pcl/kdtree/kdtree_flann.h>

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

ros::Publisher pub, raw_pub, pfh_pub, pub_2, pub_transformed;
ros::Subscriber sub;
sensor_msgs::PointCloud2 raw, out, pfh_cloud, transformed;
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

//icp
int max_iteration = 50;
double max_corresDist = 0.8f; //0.04 seems to in local optimal, converge to initial sac result (4 cm tight threshold)  >0.4 good太小無法配準
double EucFitnessEps = 0.05f; //score ->所有點距離均方差 0.2->0.05好一些 ->0.01跟0.05差不多
double TransEps = 1e-10f;

template<class T>
string ConvertToString(T value){
  stringstream ss;
  ss << value;
  return ss.str();
}


// template<class T>
// template<typename T>
template<typename T1, typename T2>
Eigen::Matrix4f CalSACia(T1 source_des, T1 target_des, pcl::PointCloud<pcl::PointXYZ>::Ptr source_temp, pcl::PointCloud<pcl::PointXYZ>::Ptr target_temp, T2 points){
  pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, T2> sac_ia;
  pcl::PointCloud <pcl::PointXYZ>::Ptr final(new pcl::PointCloud<pcl::PointXYZ> );
  Eigen::Matrix4f sac_trans = Eigen::Matrix4f::Identity();
  sac_ia.setInputSource(source_temp);
  sac_ia.setSourceFeatures(source_des);
  sac_ia.setInputTarget(target_temp);
  sac_ia.setTargetFeatures(target_des);

  pcl::console::TicToc timer_align;
  timer_align.tic();

  sac_ia.setNumberOfSamples(3);
  sac_ia.setCorrespondenceRandomness(6);//设置计算协方差时选择多少近邻点，该值越大，协防差越精确，但是计算效率越低.(可省)
  //sac_ia.setMaximumIterations(100);
  sac_ia.setEuclideanFitnessEpsilon(0.001);
  sac_ia.setTransformationEpsilon(1e-10);
  sac_ia.setRANSACIterations(30); 
  sac_ia.align(*final); //source cloud(keypoint source) aligned to target
  cout <<"has converged: "<< sac_ia.hasConverged() <<",score: "<<sac_ia.getFitnessScore()<< endl;
  cout<<"Tranformation is\n "<<sac_ia.getFinalTransformation()<<endl;
  string sac_time = ConvertToString(timer_align.toc()/1000);
  string sac_score = ConvertToString(sac_ia.getFitnessScore());
  cout <<"Need "<<timer_align.toc()/1000<<" secs.\n";
  cout<< "The final size: "<<final->points.size()<<endl;
  
  sac_trans = sac_ia.getFinalTransformation();

  return sac_trans;
} 



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
    int total_pt = (int)(cloud_in->size());
    // for(i=0;i<iteration;i++){
    for(i=0; cloud_in->size() > 0.7*total_pt; i++){
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
        cout<<i<<endl;
        cout<<cloud_in->size()<<endl;
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

pcl::PointCloud<pcl::FPFHSignature33>::Ptr Compute_FPFH(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_raw, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, int k_search, bool output){
    // Object for storing the normals.
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_src(new pcl::PointCloud<pcl::FPFHSignature33>());


    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
    normalEstimation.setInputCloud(cloud_raw);
    // 若用半徑,則要夠大 (dataset_dependent),否則可能出現normal nan  導致後面算不出來
    normalEstimation.setRadiusSearch(0.3); //original 0.03 = 3cm too small to evaluate local normal
    // normalEstimation.setKSearch(k_search);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
    normalEstimation.setSearchMethod(kdtree);
    normalEstimation.compute(*normals);


    // 移除那些infinite normal的點,不去算,剩下有normal的當作setSearchSurface （還是給值？） <-nan normal導致fpfh nan, 要移除哪部份的點？（根據fpfh/normals的nan）
    cout << normals->points.size() << endl;
    cout << normals->points[1].normal_x << endl;
    int count = 0;
    for (int i = 0; i < normals->points.size(); i++){
      if (!pcl::isFinite<pcl::Normal>(normals->points[i])){
        PCL_WARN("normals[%d] is not finite\n", i);
        cout << normals->points[i] <<endl;
        count++;
      }
    }
    PCL_WARN("We have %d infinite normal.\n",count);


    pcl::console::TicToc fpfh_time;
    fpfh_time.tic();
    pcl::FPFHEstimation<pcl::PointXYZ,pcl::Normal,pcl::FPFHSignature33> fpfh;

    fpfh.setInputCloud(cloud_in);
    fpfh.setSearchSurface(cloud_raw);
    fpfh.setRadiusSearch(0.3);
    // fpfh.setKSearch(k_search);
    fpfh.setInputNormals(normals);
    fpfh.setSearchMethod(kdtree);
    
    
    fpfh.compute(*fpfhs_src);
    std::cout<<"Computing the FPFH points takes "<<fpfh_time.toc()/1000<<"seconds"<<std::endl;
    std::cout << "The points # after FPFH: "<< fpfhs_src->points.size()<< std::endl;
    // for (int i=0; i<33; i++)
    //   std::cout << fixed << setprecision(4) << fpfhs_src->points[4].histogram[i] << std::endl;


    // output descriptor
    if (output){
      stringstream kk;
      kk << k_search;
      string filename = filename_dir + "/FPFH_k=" + kk.str() + ".csv";
      outputFile.open(filename);
      for (int k=0; k<fpfhs_src->points.size(); k++){
        // outputFile << cloud_raw->header.stamp ;
        outputFile <<  fpfhs_src->points[k].histogram[0] ;
        for(int j=1; j<33; j++){
          outputFile << "," << fpfhs_src->points[k].histogram[j] ;
        }
        outputFile << endl;
      }
      cout << "Done recording frame " << cloud_raw->header.stamp << endl;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*fpfhs_src, *cloud_out);
    return fpfhs_src;
}



pcl::PointCloud<pcl::PFHSignature125>::Ptr Compute_PFH(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_raw, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, int k_search, bool output){
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
    std::cout<<"Computing the PFH points takes "<<pfh_time.toc()/1000<<" seconds"<<std::endl;
    std::cout << "The points # after PFH: "<< descriptors->points.size()<< std::endl;
    // for (int i=0; i<125; i++)
    //   std::cout << fixed << setprecision(4) << descriptors->points[4].histogram[i] << std::endl;


    // output descriptor
    if (output){
      stringstream kk;
      kk << k_search;
      string filename = filename_dir + "/PFH_k=" + kk.str() + ".csv";
      outputFile.open(filename);
      for (int k=0; k<descriptors->points.size(); k++){
        // outputFile << cloud_raw->header.stamp ;
        outputFile <<  descriptors->points[k].histogram[0] ;
        for(int j=1; j<125; j++){
          outputFile << "," << descriptors->points[k].histogram[j] ;
        }
        outputFile << endl;
      }
      cout << "Done recording frame " << cloud_raw->header.stamp << endl;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*descriptors, *cloud_out);
    return descriptors;

}

pcl::PointCloud<pcl::SHOT352>::Ptr Compute_SHOT(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_raw, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, int k_search, bool output){
   // Object for storing the normals.
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::SHOT352>::Ptr shot_src(new pcl::PointCloud<pcl::SHOT352>());


    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
    normalEstimation.setInputCloud(cloud_raw);
    // 若用半徑,則要夠大 (dataset_dependent),否則可能出現normal nan  導致後面算不出來
    normalEstimation.setRadiusSearch(0.3); 
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
    normalEstimation.setSearchMethod(kdtree);
    normalEstimation.compute(*normals);


    // 移除那些infinite normal的點,不去算,剩下有normal的當作setSearchSurface （還是給值？） <-nan normal導致fpfh nan, 要移除哪部份的點？（根據fpfh/normals的nan）
    cout << normals->points.size() << endl;
    cout << normals->points[1].normal_x << endl;
    int count = 0;
    for (int i = 0; i < normals->points.size(); i++){
      if (!pcl::isFinite<pcl::Normal>(normals->points[i])){
        PCL_WARN("normals[%d] is not finite\n", i);
        count++;
      }
    }
    PCL_WARN("We have [%d] infinite normal.",count);


    pcl::console::TicToc shot_time;
    shot_time.tic();
    pcl::SHOTEstimation<pcl::PointXYZ,pcl::Normal,pcl::SHOT352> shot;

    shot.setInputCloud(cloud_in);
    shot.setSearchSurface(cloud_raw);
    shot.setRadiusSearch(0.3);
    // shot.setKSearch(k_search);
    shot.setInputNormals(normals);
    shot.setSearchMethod(kdtree);
    
    
    shot.compute(*shot_src);
    std::cout<<"Computing the SHOT points takes "<<shot_time.toc()/1000<<"seconds"<<std::endl;
    std::cout << "The points # after SHOT: "<< shot_src->points.size()<< std::endl;

    // output descriptor
    if (output){
      stringstream kk;
      kk << k_search;
      string filename = filename_dir + "/SHOT_k=" + kk.str() + ".csv";
      outputFile.open(filename);
      for (int k=0; k<shot_src->points.size(); k++){
        // outputFile << cloud_raw->header.stamp ;
        outputFile <<  shot_src->points[k].descriptor[0] ;
        for(int j=1; j<352; j++){
          outputFile << "," << shot_src->points[k].descriptor[j] ;
        }
        outputFile << endl;
      }
      cout << "Done recording frame " << cloud_raw->header.stamp << endl;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*shot_src, *cloud_out);
    return shot_src;
}

pcl::PointCloud<pcl::PointWithScale> do_sift(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float min_scale, int n_octaves, int n_scales_per_octave, float min_contrast){
    pcl::console::TicToc time;
    time.tic();
    // Estimate the sift interest points using z values from xyz as the Intensity variants
    pcl::SIFTKeypoint<pcl::PointXYZ, pcl::PointWithScale> sift;
    pcl::PointCloud<pcl::PointWithScale> result;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ> ());
    sift.setSearchMethod(tree);
    sift.setScales(min_scale, n_octaves, n_scales_per_octave);
    sift.setMinimumContrast(min_contrast);
    sift.setInputCloud(cloud);
    sift.compute(result);
    std::cout<<"Computing the SIFT points takes "<<time.toc()/1000<<"seconds"<<std::endl;
    std::cout << "No. of SIFT points in the result are " << result.points.size () << std::endl;

    return result;
}

Eigen::Matrix4f do_icp (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target, Eigen::Matrix4f sac_trans){
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  pcl::PointCloud <pcl::PointXYZ>::Ptr icp_result (new pcl::PointCloud <pcl::PointXYZ> );
  pcl::console::TicToc timer_icp;
  timer_icp.tic();
  icp.setInputSource(cloud_source);
  icp.setInputTarget(cloud_target);
  icp.setMaximumIterations(max_iteration);
  icp.setMaxCorrespondenceDistance(max_corresDist);
  icp.setEuclideanFitnessEpsilon(EucFitnessEps);
  icp.setTransformationEpsilon(TransEps);
  icp.align(*icp_result, sac_trans);
  cout <<"----------------------\nFor "<<method<<"\nICP:";
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

void callback(const sensor_msgs::PointCloud2 &msg){
    cout << "Get new at " << msg.header.stamp << std::endl;
    std::cout << "Original points: " << msg.width*msg.height <<std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_msg (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(msg, *cloud_msg);

    // cloud_in = filter_ground(cloud_msg);
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
    pcl::PointCloud<pcl::PFHSignature125>::Ptr pfh_feature (new pcl::PointCloud<pcl::PFHSignature125>);
    pfh_feature= Compute_PFH(cloud_in,cloud_temp, 20, false);
    pcl::copyPointCloud(*pfh_feature, *cloud_des);


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
    pub_transformed = nh.advertise<sensor_msgs::PointCloud2>("transformed", 1000);
    
    sub = nh.subscribe("/scan",1,&callback);

    signal(SIGINT, MySigintHandler);

    // string k_search;
    nh.getParam("method", method);
    // nh.getParam("k",k_search);
    // cout << k_search << endl;
    // int k_search_no = std::strtol(k_search.c_str(),nullptr,10);
    int k_search_no = 20;


  //////////////////////////////single frame
    // std::string ply_file = "/home/d300/catkin_carol/lidar_frame/PC_315966449519192000.ply";
    std::string scene_path = "/home/d300/catkin_carol/src/object_detection/segmented car from scene/";
    // std::string scene_file = "/home/d300/catkin_carol/src/object_detection/model/3D_Models/Audi_R8/scan/whole_r8.pcd";
    // std::string pcd_path = "/home/d300/catkin_carol/src/object_detection/model/3D_Models/audi_a3/scan/";
    std::string model_path = "/home/d300/catkin_carol/src/object_detection/model/3D_Models/Audi_R8/scan/temp/";
    // std::string model_file = pcd_path + argv[1];
    std::string model_file, model, scene_file, scene;
    nh.getParam("model",model);
    nh.getParam("scene",scene);
    model_file = model_path + model;
    scene_file = scene_path + scene;

    
    // std::cout << "Reading " << ply_file << std::endl;
    std::cout << "Reading " << scene_file << std::endl;
    std::cout << "Reading " << model_file << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target (new pcl::PointCloud<pcl::PointXYZ>);
    if(pcl::io::loadPCDFile<pcl::PointXYZ> (model_file, *cloud_target) == -1) // load the file
    {
      PCL_ERROR ("Couldn't read file ",model_file);
      return -1;
    }
    if(pcl::io::loadPCDFile<pcl::PointXYZ> (scene_file, *cloud_source) == -1) // load the file
    {
      PCL_ERROR ("Couldn't read file ",scene_file);
      return -1;
    }

    std::cout << "Point # of source : " << cloud_source->points.size () <<std::endl;
    std::cout << "Point # of model : " << cloud_target->points.size () <<std::endl;
    // cloud_source = filter_ground(cloud_source);
    std::cout<<"After remove ground: " << cloud_source->points.size() << std::endl;
    
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

    pcl::PointCloud<pcl::PointWithScale> result_source, result_target;
    result_source = do_sift(cloud_source, min_scale, n_octaves, n_scales_per_octave, min_contrast);
    result_target = do_sift(cloud_target, min_scale, n_octaves, n_scales_per_octave, min_contrast);

    // Copying the pointwithscale to pointxyz so as visualize the cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_temp (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_temp (new pcl::PointCloud<pcl::PointXYZ>);
    copyPointCloud(result_source, *source_temp);
    copyPointCloud(result_target, *target_temp);

    
    pcl::PointCloud <pcl::PointXYZ>::Ptr ndt_result (new pcl::PointCloud <pcl::PointXYZ> );
    Eigen::Matrix4f sac_trans = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f icp_trans = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f ndt_trans = Eigen::Matrix4f::Identity();

    //傳回time要畫在viewer上
    string icp_time, sac_time, sac_score, icp_score, ndt_time, ndt_score;

    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;

    if ( !(method.compare("pfh")) ){
      pcl::PointCloud<pcl::PFHSignature125>::Ptr pfh_source (new pcl::PointCloud<pcl::PFHSignature125>);
      pcl::PointCloud<pcl::PFHSignature125>::Ptr pfh_target (new pcl::PointCloud<pcl::PFHSignature125>);
      pfh_source = Compute_PFH(cloud_source,source_temp, k_search_no, false);
      pfh_target = Compute_PFH(cloud_target,target_temp, k_search_no, false);

      sac_trans = CalSACia(pfh_source, pfh_target, source_temp, target_temp, pfh_target->points[0]);
      /*
      sac_ia_pfh.setInputSource(source_temp);
      sac_ia_pfh.setSourceFeatures(pfh_source);
      sac_ia_pfh.setInputTarget(target_temp);
      sac_ia_pfh.setTargetFeatures(pfh_target);

      pcl::console::TicToc timer_align;
      timer_align.tic();

      sac_ia_pfh.setNumberOfSamples(3);
      sac_ia_pfh.setCorrespondenceRandomness(6);//设置计算协方差时选择多少近邻点，该值越大，协防差越精确，但是计算效率越低.(可省)
      //sac_ia.setMaximumIterations(100);
      sac_ia_pfh.setEuclideanFitnessEpsilon(0.001);
      sac_ia_pfh.setTransformationEpsilon(1e-10);
      sac_ia_pfh.setRANSACIterations(30); 
      sac_ia_pfh.align(*final); //source cloud(keypoint source) aligned to target
      cout <<"has converged: "<< sac_ia_pfh.hasConverged() <<",score: "<<sac_ia_pfh.getFitnessScore()<< endl;
      cout<<"Tranformation is\n "<<sac_ia_pfh.getFinalTransformation()<<endl;
      sac_time = ConvertToString(timer_align.toc()/1000);
      sac_score = ConvertToString(sac_ia_pfh.getFitnessScore());
      cout <<"Need "<<timer_align.toc()/1000<<" secs.\n";
      cout<< "The final size: "<<final->points.size()<<endl;
      
      sac_trans = sac_ia_pfh.getFinalTransformation();
      */

      // icp
      pcl::console::TicToc timer_icp;
      timer_icp.tic();
      icp.setInputSource(cloud_source);
      // icp.setInputSource(source_temp);
      icp.setInputTarget(cloud_target);
      icp.setMaximumIterations(50);
      icp.setMaxCorrespondenceDistance(0.4);//0.04 seems to in local optimal, converge to initial sac result (4 cm tight threshold) 
      icp.setEuclideanFitnessEpsilon(0.05);
      icp.setTransformationEpsilon(1e-10);
      icp.align(*icp_result, sac_trans);
      cout <<"----------------------\nFor "<<method<<"\nICP:";
      cout <<"has converged: "<< icp.hasConverged() <<",score: "<<icp.getFitnessScore()<< endl;
      cout<<"ICP Tranformation is\n "<<icp.getFinalTransformation()<<endl;
      double icp_times_up = timer_icp.toc()/1000.0;
      icp_time = ConvertToString(icp_times_up);
      icp_score = ConvertToString(icp.getFitnessScore());
      cout <<"Need "<<icp_times_up<<" secs.\n";
      cout<< "The final size: "<<icp_result->points.size()<<endl;

      icp_trans = icp.getFinalTransformation();

    }
    else if( !(method.compare("fpfh")) ){
      pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_source (new pcl::PointCloud<pcl::FPFHSignature33>);
      pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_target (new pcl::PointCloud<pcl::FPFHSignature33>);
      fpfh_source = Compute_FPFH(cloud_source,source_temp, k_search_no, true);
      fpfh_target = Compute_FPFH(cloud_target,target_temp, k_search_no, false);

      sac_trans = CalSACia(fpfh_source, fpfh_target, source_temp, target_temp, fpfh_target->points[0]);


      // icp
      icp_trans = do_icp(cloud_source, cloud_target, sac_trans);
      pcl::console::TicToc timer_icp;
      timer_icp.tic();
      icp.setInputSource(cloud_source);
      icp.setInputTarget(cloud_target);
      icp.setMaximumIterations(max_iteration);
      icp.setMaxCorrespondenceDistance(max_corresDist);
      icp.setEuclideanFitnessEpsilon(EucFitnessEps);
      icp.setTransformationEpsilon(TransEps);
      icp.align(*icp_result, sac_trans);
      cout <<"----------------------\nFor "<<method<<"\nICP:";
      cout <<"has converged: "<< icp.hasConverged() <<",score: "<<icp.getFitnessScore()<< endl;
      cout << icp.getConvergeCriteria()<<endl;
      
      cout<<"ICP Tranformation is\n "<<icp.getFinalTransformation()<<endl;
      double icp_times_up = timer_icp.toc()/1000.0;
      icp_time = ConvertToString(icp_times_up);
      icp_score = ConvertToString(icp.getFitnessScore());
      cout <<"Need "<<icp_times_up<<" secs.\n";
      cout<< "The final size: "<<icp_result->points.size()<<endl;

      icp_trans = icp.getFinalTransformation();

      //ndt
      pcl::console::TicToc ndt_timer;
      ndt_timer.tic();
      ndt.setTransformationEpsilon(0.01);
      ndt.setStepSize(0.1);
      ndt.setResolution(1.0);
      ndt.setMaximumIterations(35);
      ndt.setInputCloud(cloud_source);
      ndt.setInputTarget(cloud_target);
      ndt.align(*ndt_result, sac_trans);

      cout<<"NDT Transformation is\n "<<ndt.getFinalTransformation()<<endl;
      double ndt_times_up = ndt_timer.toc()/1000.0;
      std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged()
        << " score: " << ndt.getFitnessScore() << std::endl;
      ndt_time = ConvertToString(ndt_times_up);
      ndt_score = ConvertToString(ndt.getFitnessScore());
      cout<<"NDT needs " + ndt_time +" secs"<<endl;
      

      ndt_trans = ndt.getFinalTransformation(); 
      
    }
    else if (!(method.compare("shot"))){
      pcl::PointCloud<pcl::SHOT352>::Ptr shot_source (new pcl::PointCloud<pcl::SHOT352>);
      pcl::PointCloud<pcl::SHOT352>::Ptr shot_target (new pcl::PointCloud<pcl::SHOT352>);
      shot_source = Compute_SHOT(cloud_source,source_temp, k_search_no, true);
      shot_target = Compute_SHOT(cloud_target,target_temp, k_search_no, false);
      
      sac_trans = CalSACia(shot_source, shot_target, source_temp, target_temp, shot_target->points[0]);

      


      // icp
      pcl::console::TicToc timer_icp;
      timer_icp.tic();
      icp.setInputSource(cloud_source);
      // icp.setInputSource(source_temp);
      icp.setInputTarget(cloud_target);
      icp.setMaximumIterations(100);
      icp.setMaxCorrespondenceDistance(0.4);//0.04 seems to in local optimal, converge to initial sac result (4 cm tight threshold) 
      icp.setEuclideanFitnessEpsilon(0.05);//0.2
      icp.setTransformationEpsilon(1e-10);
      icp.align(*icp_result, sac_trans);
      cout <<"----------------------\nFor "<<method<<"\nICP:";
      cout <<"has converged: "<< icp.hasConverged() <<",score: "<<icp.getFitnessScore()<< endl;
      cout<<"ICP Tranformation is\n "<<icp.getFinalTransformation()<<endl;
      double icp_times_up = timer_icp.toc()/1000.0;
      icp_time = ConvertToString(icp_times_up);
      icp_score = ConvertToString(icp.getFitnessScore());
      cout <<"Need "<<icp_times_up<<" secs.\n";
      cout<< "The final size: "<<icp_result->points.size()<<endl;

      icp_trans = icp.getFinalTransformation();

    }
    else
      cout << "Just do sift.\n";
  

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_sac (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_icp_key (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_sac_key (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_ndt (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud (*cloud_source, *transformed_sac, sac_trans);
    pcl::transformPointCloud (*cloud_source, *transformed_cloud, icp_trans);
    pcl::transformPointCloud (*source_temp, *transformed_icp_key, icp_trans);
    pcl::transformPointCloud (*source_temp, *transformed_sac_key, sac_trans);
    pcl::transformPointCloud (*cloud_source, *transformed_ndt, ndt_trans);
  
    
    

    //可视化
  string win_name;
  if( !(method.compare("pfh")) )
    win_name = "pfh descriptor";
  else if ( !(method.compare("fpfh")) )
    win_name = "fpfh descriptor";
  else if ( !(method.compare("shot")) )
    win_name = "shot descriptor";
  else
    win_name = "Sift keypoint";
  
	boost::shared_ptr<pcl::visualization::PCLVisualizer> view(new pcl::visualization::PCLVisualizer(win_name));
	int v1 = 0;
	int v2 = 1;
	// view->createViewPort(0, 0, 0.5, 1, v1);
	// view->createViewPort(0.5, 0, 1, 1, v2);
  view->createViewPort(0, 0, 1, 1, v2);
	// view->setBackgroundColor(0, 0, 0, v1);
	view->setBackgroundColor(0.05, 0, 0, v2);

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color(cloud_source, 255, 0, 0);
	// view->addPointCloud(cloud_source, source_cloud_color, "sources_cloud_v1", v1);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_cloud_color(cloud_target, 255, 255, 255);
	// view->addPointCloud(cloud_target, target_cloud_color, "target_cloud_v1", v1);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_temp_color(target_temp, 0, 255, 0);
	// view->addPointCloud(target_temp, target_temp_color, "target_temp_v1", v1);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_temp_color(source_temp, 255, 255, 0);
  // view->addPointCloud(source_temp, source_temp_color, "source_temp_v1", v1);
	// view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sources_cloud_v1", v1);
  // view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target_cloud_v1", v1);
  // view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "target_temp_v1", v1);
  // view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "source_temp_v1", v1);

	// final 為 source 的 keypoint 不是原完整點雲
	// pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> aligend_cloud_color(final, 255, 0, 0);
	// view->addPointCloud(final, aligend_cloud_color, "aligend_cloud_v2", v2);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> aligend_cloud_color(transformed_cloud, 255, 0, 0);
	view->addPointCloud(transformed_cloud, aligend_cloud_color, "icp_aligend_cloud_v2", v2);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> sac_aligend_cloud_color(transformed_sac, 255, 255, 0);
	view->addPointCloud(transformed_sac, sac_aligend_cloud_color, "sac_aligend_cloud_v2", v2);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> ndt_aligend_cloud_color(transformed_ndt, 0, 0, 255);
	view->addPointCloud(cloud_target, target_cloud_color, "target_cloud_v2", v2);
  view->addPointCloud(transformed_ndt, ndt_aligend_cloud_color, "ndt_aligend_cloud_v2", v2);
	view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "icp_aligend_cloud_v2");
	view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target_cloud_v2");
  view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sac_aligend_cloud_v2");
  view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "ndt_aligend_cloud_v2");

  view->addPointCloud(target_temp, target_temp_color, "target_temp_v2", v2);
  view->addPointCloud(transformed_icp_key, source_temp_color, "source_temp_v2", v2);
  view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "target_temp_v2", v2);
  view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "source_temp_v2", v2);


  view->addText("Descriptor: "+method, 10, 50, "descriptor", v2);
  view->addText("SAC time: "+sac_time+" ,score: "+sac_score, 10, 35, "sac_time", v2);
  view->addText("ICP time: "+icp_time+" ,score: "+icp_score, 10, 20, "icp_time", v2);
  view->addText("NDT time: "+ndt_time+" ,score: "+ndt_score, 10, 5, "ndt_time", v2);
  view->addText("ICP setting :\n\nMaxIter: "+ConvertToString(max_iteration) + \
              "\n\nMax_corresDist: "+ConvertToString(max_corresDist)+ \
              "\n\nEucFitnessEps: "+ConvertToString(EucFitnessEps)+ \
              "\n\nTransEps: "+ConvertToString(TransEps), 10, 425,"Icp setting", v2);
  

	//对应关系可视化 Get correspondence
  // if ( !(method.compare("pfh")) ){
  //   Cal_correspondence(method, pfh_source, pfh_source);
  // }
  // else if( !(method.compare("pfh")) ){
  //   Cal_correspondence(method, pfh_source, pfh_source);
  // }
  
  if ( !(method.compare("pfh")) ){
    pcl::registration::CorrespondenceEstimation<pcl::PFHSignature125, pcl::PFHSignature125> crude_cor_est;
    boost::shared_ptr<pcl::Correspondences> cru_correspondences(new pcl::Correspondences);
    crude_cor_est.setInputSource(pfh_source);
    crude_cor_est.setInputTarget(pfh_target);
    //crude_cor_est.determineCorrespondences(*cru_correspondences);
    crude_cor_est.determineReciprocalCorrespondences(*cru_correspondences, 0.25f);
    cout << "crude size is " << cru_correspondences->size() << endl;
    
    for (int i=0; i<cru_correspondences->size(); i++){
      float dist = cru_correspondences->at(i).distance;
      int src_index = cru_correspondences->at(i).index_query;
      int tgt_index = cru_correspondences->at(i).index_match;
      cout << source_temp->points.at(src_index).x << ", " << source_temp->points.at(src_index).y << ", " << source_temp->points.at(src_index).z<< endl;
      cout << target_temp->points.at(tgt_index).x << ", " << target_temp->points.at(tgt_index).y << ", " << target_temp->points.at(tgt_index).z<< endl;
      cout << "dist = " << dist <<endl;
      cout << "........\n";
    }
    view->addCorrespondences<pcl::PointXYZ>(source_temp, target_temp, *cru_correspondences,"correspondence", v1);
    view->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1.5, "correspondence", v1);
    view->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,1,0,1,"correspondence");
    view->initCameraParameters();
  }
  else if(!(method.compare("fpfh"))){
    pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> crude_cor_est;
    boost::shared_ptr<pcl::Correspondences> cru_correspondences(new pcl::Correspondences);
    crude_cor_est.setInputSource(fpfh_source);
    crude_cor_est.setInputTarget(fpfh_target);
    //crude_cor_est.determineCorrespondences(*cru_correspondences);
    crude_cor_est.determineReciprocalCorrespondences(*cru_correspondences, 0.25f);

    // 用k-nearest(k=1)找model和scene descriptor 的配對（也是找最相似的descriptor)
    // pcl::KdTreeFLANN<pcl::FPFHSignature33> k_search;
    // k_search.setInputCloud(fpfh_target);

    // for(int i=0; i<fpfh_source->size(); i++){
    //   std::vector<int> neigh_indices (1);   //設置最近鄰點的索引
    //   std::vector<float> neigh_sqr_dists (1); //申明最近鄰平方距離值
    //   int found_neighs = k_search.nearestKSearch (fpfh_source->at(i), 1, neigh_indices, neigh_sqr_dists);
    //   //scene_descriptors->at (i)是給定點雲 1是臨近點個數 ，neigh_indices臨近點的索引  neigh_sqr_dists是與臨近點的索引

    //   if(found_neighs == 1 && neigh_sqr_dists[0] < 0.25f) // 僅當描述子與臨近點的平方距離小於0.25（描述子與臨近的距離在一般在0到1之間）才添加匹配
    //   {
    //     //neigh_indices[0]給定點，  i  是配準數  neigh_sqr_dists[0]與臨近點的平方距離
    //     pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
    //     cru_correspondences->push_back (corr);   //把配準的點存儲在容器中
    //   }
    // }

    cout << "crude size is " << cru_correspondences->size() << endl;

    for (int i=0; i<cru_correspondences->size(); i++){
      float dist = cru_correspondences->at(i).distance;
      int src_index = cru_correspondences->at(i).index_query;
      int tgt_index = cru_correspondences->at(i).index_match;
      cout << source_temp->points.at(src_index).x << ", " << source_temp->points.at(src_index).y << ", " << source_temp->points.at(src_index).z<< endl;
      cout << target_temp->points.at(tgt_index).x << ", " << target_temp->points.at(tgt_index).y << ", " << target_temp->points.at(tgt_index).z<< endl;
      cout << "dist = " << dist <<endl;
      cout << "........\n";
    }
  
    view->addCorrespondences<pcl::PointXYZ>(source_temp, target_temp, *cru_correspondences,"correspondence", v1);
    view->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1.5, "correspondence", v1);
    view->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,1,0,1,"correspondence");
    view->initCameraParameters();
    
  }

  pcl::toROSMsg(*cloud_source, raw);
  pcl::toROSMsg(*transformed_cloud, transformed);
  raw.header.frame_id = "scan";
  transformed.header.frame_id = "scan";
  

	while (!view->wasStopped())
	{
		view->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    raw_pub.publish(raw);
    pub_transformed.publish(transformed);
    
	}

    return 0;
  
}