#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/flann.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/io.h>
#include <iostream>
#include <vector>
#include <string>
#include <visualization_msgs/Marker.h>


using namespace std;

int main (int argc, char** argv){

    ros::init(argc, argv, "kdSearch");
    ros::NodeHandle nh("~");
    
    ros::Publisher pub_extract = nh.advertise<sensor_msgs::PointCloud2> ("pc_extract",1000);
    ros::Publisher pub_raw = nh.advertise<sensor_msgs::PointCloud2> ("pc_raw",1000);
    ros::Publisher pub_search_pt = nh.advertise<visualization_msgs::Marker>("search_pt",1000);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr extract_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if (argc < 3){
        cout<<"Please provide the absolute path of ply file and search radius."<<endl;
        return -1;
    }

    string filepath = argv[1];
    double radius = strtod(argv[2], nullptr);

    if(pcl::io::loadPLYFile<pcl::PointXYZ>(filepath, *cloud)==-1){
        PCL_ERROR("Couldn't read file");
        return -1;
    }
    
    // pcl::PointXYZ search_pt = pcl::PointXYZ (cloud->sensor_origin_[0], 
    //                                         cloud->sensor_origin_[1], 
    //                                         cloud->sensor_origin_[2]);
    pcl::PointXYZ search_pt = pcl::PointXYZ (7.0f, -3.5f, 0.0f);
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);


    // Neighbors within radius search
    vector<int> pointIdxRadiusSearch;
    vector<float> pointRadiusSquaredDistance;

    if ( kdtree.radiusSearch(search_pt, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance)<=0 ){
        cout<<"There are no nearest points.\n";
        return 0;
        //   for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
        //         cout << "    "  <<   cloud->points[ pointIdxRadiusSearch[i] ].x 
        //         << " " << cloud->points[ pointIdxRadiusSearch[i] ].y 
        //         << " " << cloud->points[ pointIdxRadiusSearch[i] ].z 
        //         << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
    }

    extract_cloud->width = pointIdxRadiusSearch.size();
    extract_cloud->height = 1;
    for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i){
        pcl::PointXYZ pt = pcl::PointXYZ(cloud->points[ pointIdxRadiusSearch[i] ].x, 
                                        cloud->points[ pointIdxRadiusSearch[i] ].y,
                                        cloud->points[ pointIdxRadiusSearch[i] ].z);
        extract_cloud->points.push_back(pt);
    }
    cout << "Find "<<extract_cloud->points.size() <<" points.\n";
    cout << pointIdxRadiusSearch.size() << endl;

    
    // // K nearest neighbor search
    // int K = 10;

    // std::vector<int> pointIdxNKNSearch(K);
    // std::vector<float> pointNKNSquaredDistance(K);

    // std::cout << "K nearest neighbor search at (" << searchPoint.x 
    //             << " " << searchPoint.y 
    //             << " " << searchPoint.z
    //             << ") with K=" << K << std::endl;

    // if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
    // {
    //     for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
    //     std::cout << "    "  <<   cloud->points[ pointIdxNKNSearch[i] ].x 
    //                 << " " << cloud->points[ pointIdxNKNSearch[i] ].y 
    //                 << " " << cloud->points[ pointIdxNKNSearch[i] ].z 
    //                 << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
    // }


    sensor_msgs::PointCloud2 extract_pc, raw_pc;
    pcl::toROSMsg(*extract_cloud, extract_pc);
    pcl::toROSMsg(*cloud,raw_pc);
    extract_pc.header.frame_id ="scan";
    raw_pc.header.frame_id="scan";

    visualization_msgs::Marker m;
    m.header.frame_id = "scan";
    m.action = visualization_msgs::Marker::ADD;
    m.type = visualization_msgs::Marker::POINTS;
    geometry_msgs::Point pt;
    pt.x = search_pt.x;
    pt.y = search_pt.y;
    pt.z = search_pt.z;
    m.points.push_back(pt);
    m.color.r = 1.0;
    m.color.b = 1.0;
    m.color.a = 1.0;
    m.scale.x = 0.3f;
    m.scale.y = 0.3f;

    stringstream ss;
    ss << radius;
    string output_name = "extracted_radius_" + ss.str() + ".pcd";
    pcl::io::savePCDFile(output_name, *extract_cloud, false);


    ros::Rate r (1);
    while(ros::ok()){
        ros::spinOnce();
        pub_raw.publish(raw_pc);
        pub_extract.publish(extract_pc);
        pub_search_pt.publish(m);
        r.sleep();
        
    }


    return 0;




}