#include<iostream>
#include<vector>
#include<pcl/point_types.h>
#include<pcl/io/pcd_io.h>
#include<sstream>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include<fstream>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <pcl/common/transforms.h>
using namespace std;
using namespace pcl;

#define SHOW_CLOUD
int
main (int argc, char** argv)
{
    
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tar (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr output (new pcl::PointCloud<pcl::PointXYZ>);
  
  pcl::io::loadPCDFile(argv[1], *cloud_src);
  pcl::io::loadPCDFile(argv[2], *cloud_tar);
  
  Eigen::Matrix4f transform_mat;
  float shifted_val = 0.1193341160 + 0.005;
  transform_mat << 0.9969376500,	0.0591009984,	-0.0512093132,	-0.0955122912,
-0.0586364921,	0.9982238900,	0.0105274199,	-0.0318814123,
0.0517405408,	-0.0074924468,	0.9986324550,	0.1193341160,
0.0000000000,	0.0000000000,	0.0000000000,	1.0000000000;
  
  pcl::transformPointCloud(*cloud_tar, *output, transform_mat);

  int vp_1 = 0, vp_2 = 1;
          
    // Create a PCLVisualizer object
  pcl::visualization::PCLVisualizer *p;
  p = new pcl::visualization::PCLVisualizer (argc, argv, "Pairwise Incremental Registration example");
  p->createViewPort (0.0, 0, 0.5, 1.0, vp_1);
  p->createViewPort (0.5, 0, 1.0, 1.0, vp_2);
  pcl::visualization::PointCloudColorHandlerCustom<PointXYZ> cloud_tar_1 (cloud_tar, 0, 255, 0);
  pcl::visualization::PointCloudColorHandlerCustom<PointXYZ> cloud_src_1 (cloud_src, 255, 0, 0); 
  p->addPointCloud (cloud_tar, cloud_tar_1, "target_1", vp_1);
  p->addPointCloud (cloud_src, cloud_src_1, "source_1", vp_1);
  
  pcl::visualization::PointCloudColorHandlerCustom<PointXYZ> cloud_tgt_h (output, 0, 255, 0);
  pcl::visualization::PointCloudColorHandlerCustom<PointXYZ> cloud_src_h (cloud_src, 255, 0, 0);       
  p->addPointCloud (output, cloud_tgt_h, "target_2", vp_2);
  p->addPointCloud (cloud_src, cloud_src_h, "source_2", vp_2);
  p->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target_2");
  while(! p->wasStopped())
  {
      p->spinOnce();
  }
          
      return 0;    
  }
