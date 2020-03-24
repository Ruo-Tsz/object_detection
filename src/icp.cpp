#include <pcl/registration/ia_ransac.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/search/kdtree.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h> 
#include <time.h>
#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/png_io.h>

using namespace std;
vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clouds;
ros::Publisher pub;
ros::Publisher pub1, pub2, pub3, pub4, pub_whole;

int main(int argc, char** argv){

    ros::init(argc, argv, "icp_model");
    ros::NodeHandle nh("~");

    pub = nh.advertise<sensor_msgs::PointCloud2>("pc",1000);
    pub1 = nh.advertise<sensor_msgs::PointCloud2>("left_front",1000);
    pub2 = nh.advertise<sensor_msgs::PointCloud2>("left_back",1000);
    pub3 = nh.advertise<sensor_msgs::PointCloud2>("right_front",1000);
    pub4 = nh.advertise<sensor_msgs::PointCloud2>("right_back",1000);
    pub_whole = nh.advertise<sensor_msgs::PointCloud2>("whole",1000);

    // string p1 = "/home/d300/catkin_carol/src/object_detection/model/3D_Models/Audi_R8/scan/scaled_left_front_3.5m00000.pcd";
    // string p2 = "/home/d300/catkin_carol/src/object_detection/model/3D_Models/Audi_R8/scan/scaled_left_back_3.5m00000.pcd";
    // string p3 = "/home/d300/catkin_carol/src/object_detection/model/3D_Models/Audi_R8/scan/scaled_right_front_3.5m00000.pcd";
    // string p4 = "/home/d300/catkin_carol/src/object_detection/model/3D_Models/Audi_R8/scan/scaled_right_back_3.5m00000.pcd";

    // string p1 = "/home/d300/catkin_carol/src/object_detection/model/3D_Models/Audi_R8/scan/left_front_3.5m_word_frame00000.pcd";
    // string p2 = "/home/d300/catkin_carol/src/object_detection/model/3D_Models/Audi_R8/scan/left_back_3.5m_word_frame00000.pcd";
    string p3 = "/home/d300/catkin_carol/src/object_detection/model/3D_Models/Audi_R8/scan/right_front_3.5m_word_frame00000.pcd";
    string p4 = "/home/d300/catkin_carol/src/object_detection/model/3D_Models/Audi_R8/scan/right_back_3.5m_word_frame00000.pcd";

    string p1 = "/home/d300/catkin_carol/src/object_detection/model/3D_Models/Audi_R8/scan/right_3.5m_word_frame00000.pcd";
    string p2 = "/home/d300/catkin_carol/src/object_detection/model/3D_Models/Audi_R8/scan/right_front_3.5m_word_frame00000.pcd";
    
    

    clouds.resize(4);
    for(int i=0; i<clouds.size(); i++){
        clouds.at(i) = boost::make_shared <pcl::PointCloud<pcl::PointXYZI>> ();
        cout<<"Done ini "<<i<<endl;
    }

    cout << "Loading...\n";
    pcl::PointCloud<pcl::PointXYZI>::Ptr temp1( new pcl::PointCloud<pcl::PointXYZI> );
    pcl::PointCloud<pcl::PointXYZI>::Ptr temp2( new pcl::PointCloud<pcl::PointXYZI> );
    pcl::PointCloud<pcl::PointXYZI>::Ptr temp3( new pcl::PointCloud<pcl::PointXYZI> );
    pcl::PointCloud<pcl::PointXYZI>::Ptr temp4( new pcl::PointCloud<pcl::PointXYZI> );

    if(pcl::io::loadPCDFile<pcl::PointXYZI> (p1, *temp1 ) == -1) // load the file
    {
      PCL_ERROR ("Couldn't read file ",p1);
      return -1;
    }
    clouds.at(0) = temp1;
    if(pcl::io::loadPCDFile<pcl::PointXYZI> (p2, *temp2 ) == -1) // load the file
    {
      PCL_ERROR ("Couldn't read file ",p2);
      return -1;
    }
    clouds.at(1) = temp2;
    if(pcl::io::loadPCDFile<pcl::PointXYZI> (p3, *temp3 ) == -1) // load the file
    {
      PCL_ERROR ("Couldn't read file ",p3);
      return -1;
    }
    clouds.at(2) = temp3;
    if(pcl::io::loadPCDFile<pcl::PointXYZI> (p4, *temp4 ) == -1) // load the file
    {
      PCL_ERROR ("Couldn't read file ",p4);
      return -1;
    }
    clouds.at(3) = temp4;
    
    

    for(int i=0; i<clouds.size(); i++){
        cout <<i<<endl;
        cout<<(clouds.at(i))->points.size()<<endl;
    }

    cout<<"prepared to icp...\n";

    Eigen::Matrix4f initial_guess = Eigen::Matrix4f::Identity();
    // initial_guess << -0.540065, -0.240687,  0.806474,   1.46473,
    //                     0.5962,  -0.78575,   0.16475,  -6.91325,
    //                     0.594034,  0.569795,  0.567853,  -6.16004,
    //                     0,         0,         0,         1;
    //icp配準
    pcl::PointCloud <pcl::PointXYZI>::Ptr cloud_src(new pcl::PointCloud<pcl::PointXYZI> );
    pcl::PointCloud <pcl::PointXYZI>::Ptr cloud_tgt(new pcl::PointCloud<pcl::PointXYZI> );
    pcl::PointCloud <pcl::PointXYZI>::Ptr icp_result(new pcl::PointCloud<pcl::PointXYZI> );
    pcl::PointCloud <pcl::PointXYZI>::Ptr final(new pcl::PointCloud<pcl::PointXYZI> );
    pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;

    for (int i=0; i<clouds.size()-1; i++){
        if ( i == 0 ){
            cloud_tgt = clouds.at(i);
            cloud_src = clouds.at(i+1);
        }
        else{
            cloud_tgt = icp_result;
            cloud_src = clouds.at(i+1);
        }

        icp.setInputSource(cloud_src);
        icp.setInputTarget(cloud_tgt);
        //Set the max correspondence distance to 4cm (e.g., correspondences with higher distances will be ignored)
        icp.setMaxCorrespondenceDistance (0.1);
        // 最大迭代次數
        icp.setMaximumIterations (50);
        // 兩次變化矩陣之間的差值
        icp.setTransformationEpsilon (1e-10);
        // 均方誤差
        icp.setEuclideanFitnessEpsilon (0.2);
        icp.align(*icp_result,initial_guess);

        // initial_guess = icp.getFinalTransformation();
        cout << "Has_converged: " << icp.hasConverged() << endl;
        cout << "Trans: \n" << initial_guess << endl;
    }
    
    sensor_msgs::PointCloud2 out,out1,out2,out3,out4;
    pcl::toROSMsg(*icp_result, out);
    pcl::toROSMsg(*(clouds.at(0)), out1);
    pcl::toROSMsg(*(clouds.at(1)), out2);
    pcl::toROSMsg(*(clouds.at(2)), out3);
    pcl::toROSMsg(*(clouds.at(3)), out4);
    out.header.frame_id = "scan";
    out1.header.frame_id = "scan";
    out2.header.frame_id = "scan";
    out3.header.frame_id = "scan";
    out4.header.frame_id = "scan";
    pub.publish(out);


    sensor_msgs::PointCloud2 out_whole;
    pcl::PointCloud<pcl::PointXYZI>::Ptr whole (new pcl::PointCloud<pcl::PointXYZI>);
    // for(int i=0; i<clouds.size(); i++){
    for(int i=0; i<2; i++){
        *whole += *(clouds.at(i)); 
    }

    pcl::io::savePCDFile("right_front_3.5m_r8.pcd", *whole, false);
    pcl::toROSMsg(*(whole), out_whole);
    out_whole.header.frame_id = "scan";
    

    ros::Rate r(1);
    while(ros::ok()){
        pub.publish(out);
        pub1.publish(out1);
        pub2.publish(out2);
        pub3.publish(out3);
        pub4.publish(out4);
        pub_whole.publish(out_whole);
        r.sleep();
    }

    
    // clock_t end=clock();
    // cout<<"total time: "<<(double)(end-start)/(double)CLOCKS_PER_SEC<<" s"<<endl;
    // //我把計算法線和點特徵直方圖的時間也算在SAC裏面了
    // cout<<"sac time: "<<(double)(sac_time-start)/(double)CLOCKS_PER_SEC<<" s"<<endl;
    // cout<<"icp time: "<<(double)(end-sac_time)/(double)CLOCKS_PER_SEC<<" s"<<endl;

    // std::cout << "ICP has converged:" << icp.hasConverged()
    //     << " score: " << icp.getFitnessScore() << std::endl;
    // Eigen::Matrix4f icp_trans;
    // icp_trans=icp.getFinalTransformation();
    // //cout<<"ransformationProbability"<<icp.getTransformationProbability()<<endl;
    // std::cout<<icp_trans<<endl;
    // //使用創建的變換對未過濾的輸入點雲進行變換
    // pcl::transformPointCloud(*cloud_src_o, *icp_result, icp_trans);
    // //保存轉換的輸入點雲
    // pcl::io::savePCDFileASCII("bunny_transformed_sac_ndt.pcd", *icp_result);

    return 0;
}