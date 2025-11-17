#ifndef __ZED_CUDA_H__
#define __ZED_CUDA_H__

#include <cuda.h>
#include <cuda_runtime.h>
#include <cmath>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

// cuda处理点云
__host__ void processPointCloud(float *p_data_cloud, size_t cloud_size, pcl::PointCloud<pcl::PointXYZRGB>::Ptr vis_cloud_return);

#endif
