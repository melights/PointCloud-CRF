#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <fstream>
#include <iostream>
#include <array>
#include <stdio.h>
#include <string.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/segmentation/crf_segmentation.h>
#include <pcl/features/normal_3d.h>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

typedef pcl::PointCloud<pcl::PointXYZRGB> CloudT;
typedef pcl::PointCloud<pcl::PointXYZRGBL> CloudLT;
//./crf source.pcd labelled.pcd 15 3 1.3 30 20 4 5 0.1 1 30

void compute (const CloudT::Ptr &cloud, 
         const CloudLT::Ptr &anno,
         float normal_radius_search,
         float leaf_x, float leaf_y, float leaf_z,
         char** argv,
         CloudLT::Ptr &out)
{
	TicToc tt;
	tt.tic ();

	print_highlight ("Computing ");

	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals (new pcl::PointCloud<pcl::PointNormal>);
	cloud_normals->width = cloud->width;
	cloud_normals->height = cloud->height;
	cloud_normals->points.resize (cloud->points.size ());
	for (size_t i = 0; i < cloud->points.size (); i++)
	{
		cloud_normals->points[i].x = cloud->points[i].x;
		cloud_normals->points[i].y = cloud->points[i].y;
		cloud_normals->points[i].z = cloud->points[i].z;
	}

	// estimate surface normals
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::PointNormal> ne;
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
	ne.setSearchMethod (tree);
	ne.setInputCloud (cloud);
	//ne.setRadiusSearch (normal_radius_search);
	ne.setKSearch(atoi(argv[3]));
	ne.compute (*cloud_normals);

	pcl::CrfSegmentation<pcl::PointXYZRGB> crf;
	crf.setInputCloud (cloud);
	crf.setNormalCloud (cloud_normals);
	crf.setAnnotatedCloud (anno);
	crf.setVoxelGridLeafSize (leaf_x, leaf_y, leaf_z);
	crf.setSmoothnessKernelParameters (atoi(argv[4]), atoi(argv[4]), atoi(argv[4]), atof(argv[5])); //2.5
	crf.setAppearanceKernelParameters (atoi(argv[6]), atoi(argv[6]), atoi(argv[6]), atoi(argv[7]), atoi(argv[7]), atoi(argv[7]), atof(argv[8])); //3.5
	crf.setSurfaceKernelParameters (atoi(argv[9]), atoi(argv[9]), atoi(argv[9]), atof(argv[10]), atof(argv[10]), atof(argv[10]), atof(argv[11])); //1.0
        std::cout<<atoi(argv[8])<<std::endl;
	crf.setNumberOfIterations (atoi(argv[12]));
	crf.segmentPoints (*out);

	print_info ("[done, "); 
	print_value ("%g", tt.toc ()); 
	print_info (" ms : "); print_value ("%d", out->width * out->height); 
	print_info (" points]\n");
}

int main(int argc, char** argv)
{

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr src (new pcl::PointCloud<pcl::PointXYZRGB>);		// original point cloud data
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr anno (new pcl::PointCloud<pcl::PointXYZRGBL>);
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr out (new pcl::PointCloud<pcl::PointXYZRGBL>);


        pcl::io::loadPCDFile<pcl::PointXYZRGB>(argv[1], *src);
        pcl::io::loadPCDFile<pcl::PointXYZRGBL>(argv[2], *anno);
        compute(src, anno, 0.005, 0.01, 0.01, 0.01, argv,out);
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
        viewer->initCameraParameters ();

        int v1;
        viewer->createViewPort (0.0, 0.0, 0.5, 1.0, v1);
        viewer->setBackgroundColor (0, 0, 0, v1);
        pcl::visualization::PointCloudColorHandlerLabelField<pcl::PointXYZRGBL> rgb (anno);
        viewer->addPointCloud<pcl::PointXYZRGBL> (anno, rgb, "source", v1);

	// pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals (new pcl::PointCloud<pcl::PointNormal>);
	// pcl::NormalEstimation<pcl::PointXYZRGB, pcl::PointNormal> ne;
	// pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
	// ne.setSearchMethod (tree);
	// ne.setInputCloud (src);
	// //ne.setRadiusSearch (normal_radius_search);
	// ne.setKSearch(atoi(argv[3]));
	// ne.compute (*cloud_normals);


        int v2;
        viewer->createViewPort (0.5, 0.0, 1.0, 1.0, v2);
        viewer->setBackgroundColor (0, 0, 0, v2);
        pcl::visualization::PointCloudColorHandlerLabelField<pcl::PointXYZRGBL> single_color (out);
        viewer->addPointCloud<pcl::PointXYZRGBL> (out, single_color, "crf refined", v2);
		//viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::PointNormal> (src, cloud_normals, 10, 0.05, "normals");
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "source");
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "crf refined");
        viewer->addCoordinateSystem (1.0);
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	pcl::io::savePCDFileASCII ("result.pcd", *out);

        return 0;
}
