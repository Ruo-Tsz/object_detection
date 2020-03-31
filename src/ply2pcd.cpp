#include <iostream>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>

#include <pcl/PolygonMesh.h>
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <pcl/io/vtk_lib_io.h>

#include<pcl/io/io.h>
#include<pcl/io/pcd_io.h>//pcd 讀寫類相關的頭文件。
#include<pcl/io/ply_io.h>
#include<pcl/point_types.h> //PCL中支持的點類型頭文件。
int user_data;
using namespace std;

void viewerOneOff(pcl::visualization::PCLVisualizer& viewer) {
	viewer.setBackgroundColor(1.0, 0.5, 1.0);   //設置背景顏色
}

int main()
{
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());//創建點雲對象
	pcl::PolygonMesh mesh;
	vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
	// pcl::io::loadPolygonFilePLY("/home/d300/catkin_carol/src/object_detection/segmented car from scene/PC_315966449519192000.ply", mesh);									//PCL利用VTK的IO接口，可以直接讀取stl,ply,obj等格式的三維點雲數據,傳給PolygonMesh對象
	// pcl::io::mesh2vtk(mesh, polydata);												//將PolygonMesh對象轉化爲vtkPolyData對象
	// pcl::io::vtkPolyDataToPointCloud(polydata, *cloud);								//獲取點雲
	// pcl::io::savePCDFileASCII("/home/d300/catkin_carol/src/object_detection/segmented car from scene/PC_315966449519192000.pcd", *cloud);									//存儲爲pcb文件

	

	char strfilepath[256] = "/home/d300/catkin_carol/src/object_detection/segmented car from scene/PC_315966449519192000.ply";
	if (-1 == pcl::io::loadPLYFile(strfilepath, *cloud)) {							//讀取pcd格式文件
		cout << "error input!" << endl;
		return -1;
	}

    pcl::io::savePCDFile("/home/d300/catkin_carol/src/object_detection/segmented car from scene/PC_315966449519192000.pcd", *cloud, false);
	cout << cloud->points.size() << endl;											//點雲裏點規模
	pcl::visualization::CloudViewer viewer("Cloud Viewer");							  //創建可視化viewer對象

	viewer.showCloud(cloud);
	viewer.runOnVisualizationThreadOnce(viewerOneOff);
	system("pause");

	return 0;
}
