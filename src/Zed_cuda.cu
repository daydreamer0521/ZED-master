#include "Zed_cuda.h"

__device__ inline float convertColor(float colorIn)
{
    uint32_t color_uint = *(uint32_t *)&colorIn;
    unsigned char *color_uchar = (unsigned char *)&color_uint;
    color_uint = ((uint32_t)color_uchar[0] << 16 | (uint32_t)color_uchar[1] << 8 | (uint32_t)color_uchar[2]);
    return *reinterpret_cast<float *>(&color_uint);
}

// CUDA核函数，用于处理点云数据
__global__ void cuda_processPointCloud(float *p_data_cloud, int cloud_size, pcl::PointXYZRGB *vis_cloud)
{
    int index = blockIdx.x * blockDim.x + threadIdx.x;

    if (index < cloud_size)
    {
        float X = p_data_cloud[index * 4];
        if (std::isfinite(X))
        {
            vis_cloud[index].x = X;
            vis_cloud[index].y = -p_data_cloud[index * 4 + 1];
            vis_cloud[index].z = -p_data_cloud[index * 4 + 2];
            vis_cloud[index].rgb = convertColor(p_data_cloud[index * 4 + 3]);
        }
        else
        {
            vis_cloud[index].x = vis_cloud[index].y = vis_cloud[index].z = vis_cloud[index].rgb = 0;
        }
    }
}

// cuda处理点云
__host__ void processPointCloud(float *p_data_cloud, size_t cloud_size, pcl::PointCloud<pcl::PointXYZRGB>::Ptr vis_cloud_return)
{
    // 将点云数据从CPU内存传输到GPU内存
    float *d_data_cloud;
    cudaMalloc((void **)&d_data_cloud, cloud_size * 4 * sizeof(float));
    cudaMemcpy(d_data_cloud, p_data_cloud, cloud_size * 4 * sizeof(float), cudaMemcpyHostToDevice);

    // 分配GPU内存用于处理后的点云数据
    pcl::PointXYZRGB *d_vis_cloud;
    cudaMalloc((void **)&d_vis_cloud, cloud_size * sizeof(pcl::PointXYZRGB));

    // 定义CUDA核函数的块和线程设置
    int threadsPerBlock = 256;
    int numBlocks = (cloud_size + threadsPerBlock - 1) / threadsPerBlock;

    // 调用CUDA核函数处理点云数据
    cuda_processPointCloud<<<numBlocks, threadsPerBlock>>>(d_data_cloud, cloud_size, d_vis_cloud);

    // 等待CUDA所有线程执行完毕
    cudaDeviceSynchronize();

    // 将处理后的点云数据从GPU内存传输回CPU内存
    cudaMemcpy(vis_cloud_return->points.data(), d_vis_cloud, cloud_size * sizeof(pcl::PointXYZRGB), cudaMemcpyDeviceToHost);

    // 释放GPU内存
    cudaFree(d_data_cloud);
    cudaFree(d_vis_cloud);
}