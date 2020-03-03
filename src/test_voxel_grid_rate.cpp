#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/console/time.h>

using namespace std;

int main(int argc, char** argv){

    std::string ply_file = "/home/d300/catkin_carol/lidar_frame/PC_315966449519192000.ply";
    std::cout << "Reading " << ply_file << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_2 (new pcl::PointCloud<pcl::PointXYZ>);
    if(pcl::io::loadPLYFile<pcl::PointXYZ> (ply_file, *cloud) == -1) // load the file
    {
      PCL_ERROR ("Couldn't read file");
      return -1;
    }

    pcl::console::TicToc time_1, time_2;
    time_1.tic();
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (0.2f, 0.2f, 0.2f);
    sor.filter (*cloud_filtered);
    cout << "The time for 0.2 : "<<time_1.toc()/1000<<" secs.\n";

    time_2.tic();
    pcl::VoxelGrid<pcl::PointXYZ> sor_2;
    sor.setInputCloud (cloud);
    sor.setLeafSize (0.5f, 0.5f, 0.5f);
    sor.filter (*cloud_filtered_2);
    cout << "The time for 0.5 : "<<time_2.toc()/1000<<" secs.\n";

    return 0;

}

 