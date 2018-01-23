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
const std::array<int,3> a_0= {0,   192,   0};
const std::array<int,3> a_1= {128,   0,   0};
const std::array<int,3> a_2= {128, 128,   0};
const std::array<int,3> a_3= {  0,  64, 140};
const std::array<int,3> a_4= {  0,   0, 128};
const std::array<int,3> a_5= {128,   0, 128};
const std::array<int,3> a_6= {  0, 128, 128};
const std::array<int,3> a_7= {128, 128, 128};
const std::array<int,3> a_8= { 64,   0,   0};
const std::array<int,3> a_9= {192,   0, 128};
const std::array<int,3> a_10= { 64, 128,   0};
const std::array<int,3> a_11= {255, 255, 255};
const std::array<int,3> a_12= { 64,   0, 128};
const std::array<int,3> a_13= {100,  70,  50};
const std::array<int,3> a_14= { 50,  70, 100};
const std::array<int,3> a_15= {192, 128, 128};
const std::array<int,3> a_16= {  0,  64,   0};
const std::array<int,3> a_17= {128,  64,   0};
const std::array<int,3> a_18= {  0, 192,   0};
const std::array<int,3> a_19= {128, 192,   0};
const std::array<int,3> a_20 = {  0,  64, 128};
const std::array<int,3> a_21= { 64, 128, 128};
const std::array<int,3> a_22= {192, 128,   0};
const std::array<int,3> a_255= {  0,   0,   0};

int decode(int r,int g, int b){

        std::array<int,3> input = {r,   g,   b};

                if (input == a_0  )  return  0;
                if (input == a_1  )  return  1;
                if (input == a_2  )  return  2;
                if (input == a_3  )  return  3;
                if (input == a_4  )  return  4;
                if (input == a_5  )  return  5;
                if (input == a_6  )  return  6;
                if (input == a_7  )  return  7;
                if (input == a_8  )  return  8;
                if (input == a_9  )  return  9;
                if (input == a_10 )  return 10;
                if (input == a_11 )  return 11;
                if (input == a_12 )  return 12;
                if (input == a_13 )  return 13;
                if (input == a_14 )  return 14;
                if (input == a_15 )  return 15;
                if (input == a_16 )  return 16;
                if (input == a_17 )  return 17;
                if (input == a_18 )  return 18;
                if (input == a_19 )  return 19;
                if (input == a_20 )  return 20;
                if (input == a_21 )  return 21;
                if (input == a_22 )  return 22;
                if (input == a_255)  return 255;

} 

int main(int argc, char** argv)
{

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud (new pcl::PointCloud<pcl::PointXYZRGB>);		// original point cloud data
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr inputCloudL (new pcl::PointCloud<pcl::PointXYZRGBL>);


        ///////////////////////////*点云载入模块*///////////////////////
        if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(argv[1], *inputCloud) == -1)
                {
                        PCL_ERROR("Couldn't read file mypointcloud.pcd\n");  //若读取失败将提示
                        return -1;
                }
        std::cout<<"Retrieved point cloud:"<< inputCloud->size() <<std::endl;

        pcl::PointXYZRGBL pointRGBL;
	for (pcl::PointCloud<pcl::PointXYZRGB>::iterator it=inputCloud->begin(); it!=inputCloud->end(); it++){
                //std::cout<<static_cast<double>(it->r) << ", " << static_cast<float>(it->g) << ", " << static_cast<short>(it->b) <<", " <<decode(static_cast<short>(it->r),static_cast<short>(it->g),static_cast<short>(it->b)) << std::endl;
                pointRGBL.x=it->x;
                pointRGBL.y=it->y;
                pointRGBL.z=it->z;
                pointRGBL.r=it->r;
                pointRGBL.g=it->g;
                pointRGBL.b=it->b;
                pointRGBL.label=decode(static_cast<short>(it->r),static_cast<short>(it->g),static_cast<short>(it->b));
                inputCloudL->points.push_back(pointRGBL);
	}
        inputCloudL->width = 1;
        inputCloudL->height = inputCloud->points.size();

        pcl::io::savePCDFile<pcl::PointXYZRGBL>(argv[2],*inputCloudL);

        return 0;
}
