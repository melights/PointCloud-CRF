#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <fstream>
#include <iostream>
#include <array>
#include <stdio.h>
#include <string.h>
#include <pcl/surface/poisson.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/mls.h>


std::array<int,3> encode(int label){

        std::array<int,3> a;

        if (label == 0  )  a= {0,   192,   0};
        if (label == 1  )  a= {128,   0,   0};
        if (label == 2  )  a= {128, 128,   0};
        if (label == 3  )  a= {  0,  64, 140};
        if (label == 4  )  a= {  0,   0, 128};
        if (label == 5  )  a= {128,   0, 128};
        if (label == 6  )  a= {  0, 128, 128};
        if (label == 7  )  a= {128, 128, 128};
        if (label == 8  )  a= { 64,   0,   0};
        if (label == 9  )  a= {192,   0, 128};
        if (label == 10 )  a= { 64, 128,   0};
        if (label == 11 )  a= {255, 255, 255};
        if (label == 12 )  a= { 64,   0, 128};
        if (label == 13 )  a= {100,  70,  50};
        if (label == 14 )  a= { 50,  70, 100};
        if (label == 15 )  a= {192, 128, 128};
        if (label == 16 )  a= {  0,  64,   0};
        if (label == 17 )  a= {128,  64,   0};
        if (label == 18 )  a= {  0, 192,   0};
        if (label == 19 )  a= {128, 192,   0};
        if (label == 20 )  a= {  0,  64, 128};
        if (label == 21 )  a= { 64, 128, 128};
        if (label == 22 )  a= {192, 128,   0};
        if (label == 255)  a= {  0,   0,   0};

        return a;

} 

int main(int argc, char** argv)
{

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud (new pcl::PointCloud<pcl::PointXYZRGB>);		// original point cloud data
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr inputCloudL (new pcl::PointCloud<pcl::PointXYZRGBL>);


        ///////////////////////////*点云载入模块*///////////////////////
        if (pcl::io::loadPCDFile<pcl::PointXYZRGBL>(argv[1], *inputCloudL) == -1)
                {
                        PCL_ERROR("Couldn't read file mypointcloud.pcd\n");  //若读取失败将提示
                        return -1;
                }
        std::cout<<"Retrieved point cloud:"<< inputCloudL->size() <<std::endl;

        pcl::PointXYZRGB pointRGB;
	for (pcl::PointCloud<pcl::PointXYZRGBL>::iterator it=inputCloudL->begin(); it!=inputCloudL->end(); it++){
                //std::cout<<static_cast<double>(it->r) << ", " << static_cast<float>(it->g) << ", " << static_cast<short>(it->b) <<", " <<decode(static_cast<short>(it->r),static_cast<short>(it->g),static_cast<short>(it->b)) << std::endl;
                std::array<int,3> rgb;
                rgb=encode(it->label);
                pointRGB.x=it->x;
                pointRGB.y=it->y;
                pointRGB.z=it->z;
                pointRGB.r=rgb[0];
                pointRGB.g=rgb[1];
                pointRGB.b=rgb[2];
                
                inputCloud->points.push_back(pointRGB);
	}
        inputCloud->width = 1;
        inputCloud->height = inputCloud->points.size();

        pcl::io::savePCDFile<pcl::PointXYZRGB>(argv[2],*inputCloud);

        return 0;
}
