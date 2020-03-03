// STL
#include <iostream>
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/png_io.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/common/float_image_utils.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

ros::Subscriber sub;

void convert_to_range(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

	std::cout<<"in range\n";
	std::cout<<cloud->points.size();
	
	// Parameters needed by the range image object:

	// Angular resolution is the angular distance between pixels.
	float angularResolution = (float) (  1.0f * (M_PI/180.0f));  //  1.0 degree in radians
	// Maximum horizontal and vertical angles. For example, for a full panoramic scan,
	// the first would be 360º. Choosing values that adjust to the real sensor will
	// decrease the time it takes, but don't worry. If the values are bigger than
	// the real ones, the image will be automatically cropped to discard empty zones.
	float maxAngleX = (float)(360.0f * (M_PI / 180.0f));
	float maxAngleY = (float)(180.0f * (M_PI / 180.0f));
	// Sensor pose. Thankfully, the cloud includes the data.
	Eigen::Affine3f sensorPose = Eigen::Affine3f(Eigen::Translation3f(cloud->sensor_origin_[0],
								 cloud->sensor_origin_[1],
								 cloud->sensor_origin_[2])) *
								 Eigen::Affine3f(cloud->sensor_orientation_);
	// Noise level. If greater than 0, values of neighboring points will be averaged.
	// This would set the search radius (e.g., 0.03 == 3cm).
	float noiseLevel = 0.0f;
	// Minimum range. If set, any point closer to the sensor than this will be ignored.
	float minimumRange = 0.0f;
	// Border size. If greater than 0, a border of "unobserved" points will be left
	// in the image when it is cropped.
	int borderSize = 1;

	std::cout<<"done setting\n";

	// Range image object.
	pcl::RangeImage rangeImage;
	rangeImage.createFromPointCloud(*cloud, angularResolution,
									maxAngleX, maxAngleY, sensorPose, pcl::RangeImage::CAMERA_FRAME,
									noiseLevel, minimumRange, borderSize);

	std::cout<<"done create\n";

	// Visualize the image.
	pcl::visualization::RangeImageVisualizer viewer("Range image");
	viewer.showRangeImage(rangeImage);
    std::cout << rangeImage << endl;
	// while (!viewer.wasStopped())
	// {
	// 	viewer.spinOnce();
	// 	// Sleep 100ms to go easy on the CPU.
	// 	pcl_sleep(0.1);
	// }
	
	exit(-1);
}


void callback(const sensor_msgs::PointCloud2 &msg){
	cout << "Get new at " << msg.header.stamp << std::endl;
	std::cout << "Original points: " << msg.width*msg.height <<std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_msg (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(msg, *cloud_msg);

	convert_to_range(cloud_msg);
}


void planar_projection(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
	// Parameters needed by the planar range image object:

	// Image size. Both Kinect and Xtion work at 640x480.
	int imageSizeX = 640;
	int imageSizeY = 480;
	// Center of projection. here, we choose the middle of the image.
	float centerX = 640.0f / 2.0f;
	float centerY = 480.0f / 2.0f;
	// Focal length. The value seen here has been taken from the original depth images.
	// It is safe to use the same value vertically and horizontally.
	float focalLengthX = 525.0f, focalLengthY = focalLengthX;
	// Sensor pose. Thankfully, the cloud includes the data.
	Eigen::Affine3f sensorPose = Eigen::Affine3f(Eigen::Translation3f(cloud->sensor_origin_[0],
								 cloud->sensor_origin_[1],
								 cloud->sensor_origin_[2])) *
								 Eigen::Affine3f(cloud->sensor_orientation_);
	// Noise level. If greater than 0, values of neighboring points will be averaged.
	// This would set the search radius (e.g., 0.03 == 3cm).
	float noiseLevel = 0.0f;
	// Minimum range. If set, any point closer to the sensor than this will be ignored.
	float minimumRange = 0.0f;

	// Planar range image object.
	pcl::RangeImagePlanar rangeImagePlanar;
	rangeImagePlanar.createFromPointCloudWithFixedSize(*cloud, imageSizeX, imageSizeY,
			centerX, centerY, focalLengthX, focalLengthX,
			sensorPose, pcl::RangeImage::CAMERA_FRAME,
			noiseLevel, minimumRange);

	// Visualize the image.
	pcl::visualization::RangeImageVisualizer viewer("Planar range image");
	viewer.showRangeImage(rangeImagePlanar);
	std::cout << rangeImagePlanar << endl;
	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
		// Sleep 100ms to go easy on the CPU.
		pcl_sleep(0.1);
	}
}

int main(int argc, char** argv)
{	
	ros::init(argc, argv, "range_img");
	ros::NodeHandle nh("~");


	/*
	sub = nh.subscribe("/nuscenes_lidar",1,&callback);
	
	ros::Rate r(1);
	while(ros::ok()){
		ros::spinOnce();
		r.sleep();
	}
	*/
	

	
	// Object for storing the point cloud.
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	// Read a PCD file from disk.
    std::string ply_file = "/home/d300/catkin_carol/lidar_frame/PC_315966449519192000.ply";
    std::cout << "Reading " << ply_file << std::endl;
	if (pcl::io::loadPLYFile<pcl::PointXYZ>(ply_file, *cloud) != 0)
	{
		return -1;
	}

	// for (float y = -0.5f; y <= 0.5f; y += 0.01f) {
	// 	for (float z = -0.5f; z <= 0.5f; z += 0.01f) {
	// 		pcl::PointXYZ point;
	// 		point.x = 2.0f - y;
	// 		point.y = y;
	// 		point.z = z;
	// 		cloud->points.push_back(point);
	// 	}
	// }
	// cloud->width = (uint32_t)cloud->points.size();
	// cloud->height = 1;


	std::cout << cloud->points.size() << std::endl;
	std::cout << cloud->sensor_origin_[0] << "," << cloud->sensor_origin_[1] << "," << cloud->sensor_origin_[2] << std::endl;

	// planar_projection(cloud);

	
	// Parameters needed by the range image object:

	// Angular resolution is the angular distance between pixels.
	float angularResolution = (float) (  1.0f * (M_PI/180.0f));  //  1.0 degree in radians
	// Maximum horizontal and vertical angles. For example, for a full panoramic scan,
	// the first would be 360º. Choosing values that adjust to the real sensor will
	// decrease the time it takes, but don't worry. If the values are bigger than
	// the real ones, the image will be automatically cropped to discard empty zones.
	float maxAngleX = (float)(360.0f * (M_PI / 180.0f));
	float maxAngleY = (float)(180.0f * (M_PI / 180.0f));
	// Sensor pose. Thankfully, the cloud includes the data.
	Eigen::Affine3f sensorPose = Eigen::Affine3f(Eigen::Translation3f(cloud->sensor_origin_[0],
								 cloud->sensor_origin_[1],
								 cloud->sensor_origin_[2])) *
								 Eigen::Affine3f(cloud->sensor_orientation_);
	// Eigen::Affine3f sensorPose = Eigen::Affine3f(Eigen::Translation3f(0.0f, 0.0f, 0.0f));
	// Noise level. If greater than 0, values of neighboring points will be averaged.
	// This would set the search radius (e.g., 0.03 == 3cm).
	float noiseLevel = 0.0f;
	// Minimum range. If set, any point closer to the sensor than this will be ignored.
	float minimumRange = 0.0f;
	// Border size. If greater than 0, a border of "unobserved" points will be left
	// in the image when it is cropped.
	int borderSize = 1;

	cout << "Done setting\n";

	// Range image object.
	pcl::RangeImage rangeImage;
	rangeImage.createFromPointCloud(*cloud, angularResolution,
									maxAngleX, maxAngleY, sensorPose, pcl::RangeImage::LASER_FRAME,
									noiseLevel, minimumRange, borderSize);

	cout << "Done create\n";




	////////extrack border
	pcl::RangeImageBorderExtractor border_extractor (&rangeImage);
  	pcl::PointCloud<pcl::BorderDescription> border_descriptions;
  	border_extractor.compute (border_descriptions);


	// check如何將border_description顯示在image上（tutorial）
	std::cout << border_descriptions.points.at(0);
	// Visualize the image.
	pcl::visualization::RangeImageVisualizer viewer("Range image");
	viewer.showRangeImage(rangeImage);
    std::cout << rangeImage << endl;

	// show point on range image
	pcl::visualization::RangeImageVisualizer* range_image_borders_widget = NULL;
  	range_image_borders_widget = pcl::visualization::RangeImageVisualizer::getRangeImageBordersWidget (rangeImage, -std::numeric_limits<float>::infinity (), std::numeric_limits<float>::infinity (), false,
                                                                          border_descriptions, "Range image with borders");


	float *range_array = rangeImage.getRangesArray();
	unsigned char* rgb_image = pcl::visualization::FloatImageUtils::getVisualImage (range_array, rangeImage.width, rangeImage.height);
	pcl::io::saveRgbPNGFile("/home/d300/catkin_carol/src/object_detection/range_image/border.png", rgb_image, rangeImage.width, rangeImage.height); 


	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
		range_image_borders_widget->spinOnce ();
		// Sleep 100ms to go easy on the CPU.
		pcl_sleep(0.1);
	}
	
}