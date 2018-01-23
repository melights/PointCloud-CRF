# PointCloud-CRF
Conditional Random Fields label refine on pointcloud.

# Requirement: 
Point Cloud Library (PCL) http://pointclouds.org/

# Build:
mkdir build
cd build
cmake ..
make

# Run:
1. convert color label to PCL pointcloud label attribute
./label orginal.pcd labelled.pcd

2. run CRF
./crf original.pcd labelled.pcd parameters

3. convert PCL label to color label
./unlabel labelled.pcd result.pcd
