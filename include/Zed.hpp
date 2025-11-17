/*
 * @Author: your name
 * @Date: 2020-12-24 08:30:32
 * @LastEditTime: 2020-12-25 15:07:23
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /ZedDriver/ZED.hpp
 */
#ifndef ZED_HPP
#define ZED_HPP

#include <iostream>

#include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/impl/io.hpp>

#include <sl/Camera.hpp>

#ifndef ZED_SN_0
#define ZED_SN_0 34023967 // 旧相机SN
#endif

#ifndef ZED_SN_1
#define ZED_SN_1 38962368 // 新相机SN
#endif

using namespace std;
using namespace cv;

class ZED
{
public:
   ZED();    // 构造函数
   ~ZED() {} // 析构函数

   /***********************设置相机***********************/
private:
   sl::Camera zed;

   static const size_t size_devList = 2;
   sl::Camera zeds[size_devList];
   sl::CameraInformation zeds_info[size_devList];

   sl::Resolution image_size;

   sl::RuntimeParameters runtime_params;

public:
   /**
    * 初始化zed相机
    */
   void init();
   void init(size_t ZED_SN);

   /**
    * 设置zed相机参数
    */
   void setCamera();
   /**
    * 设置指定SN码zed相机参数
    * @param serial_number 指定SN码
    */
   void setCamera(unsigned int serial_number);

   /**
    * 关zed相机
    */
   void close();
   void close(size_t ZED_SN);

   /***********************设置相机***********************/

   /***********************获取图片/点云***********************/
private:
   /**
    * 获取opencv图像类型
    * @param type sl图像类型
    * @return opencv图像类型
    */
   int getOCVtype(sl::MAT_TYPE type);

   /**
    * 获取opencv图像
    * @param input sl图像
    * @return opencv图像
    */
   cv::Mat slMat2cvMat(sl::Mat &input);

public:
   /**
    * 获取zed相机图像
    * @return vector<cv::Mat> pictures 表示获取到的图像； pictures[0] 彩色图像； pictures[1] 深度图像；
    */
   vector<cv::Mat> getImg();

   /**
    * 获取点云
    * @return 获取到的点云
    */
   void getCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr vis_cloud_return, cv::Mat &image_return);
   /**
    * 获取指定SN码zed点云
    * @return 获取到的点云
    */
   void getCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr vis_cloud_return, cv::Mat &image_return, unsigned int serial_number);

   /**
    * 获取像素点的深度
    * @param center 像素点的坐标
    * @return 像素点的深度
    */
   float calcDistance(cv::Point center);

   /**
    * 从像素坐标转为世界坐标
    * @param center 像素坐标
    * @param Wolrd 世界坐标； World[0] 世界坐标x； World[1] 世界坐标y； World[2] 世界坐标z；
    */
   void coordinate(cv::Point center, float *World);

   /**
    * @brief 更新sl点云，并存储在private中
    *
    */
   void update_sl_point_cloud();

   /**
    * @brief 从sl点云中获取世界坐标
    *
    * @param center 像素坐标
    * @param World 世界坐标； World[0] 世界坐标x； World[1] 世界坐标y； World[2] 世界坐标z；
    */
   void coordinate_from_sl_point_cloud(cv::Point center, float *World);

   /***********************获取图片/点云***********************/

private:
   /**
    * 获取opencv GpuMat图像
    * @param input sl图像
    * @return opencv GpuMat图像
    */
   cv::cuda::GpuMat slMat2cvMatGPU(sl::Mat &input);

   sl::Mat sl_point_cloud;

   int new_width;
   int new_height;

private:
   struct Yaml_Zed_Settings
   {
      int brightness = 4;                  // 亮度
      int contrast = 4;                    // 对比度
      int hue = 0;                         // 色调
      int saturation = 4;                  // 饱和度
      int sharpness = 4;                   // 锐度
      int gamma = 0;                       // 伽马矫正
      int gain = -1;                       // 曝光增益
      int exposure = -1;                   // 曝光时间
      int whitebalance_temperature = 4000; // 白平衡
   } yaml_zed_settings[2];

public:
   cv::cuda::GpuMat depthMask(float dist, float range, cv::cuda::GpuMat temp)
   {
      sl::Mat point_cloud2;
      sl::float1 point;
      zed.retrieveMeasure(point_cloud2, sl::MEASURE::DEPTH, sl::MEM::GPU, image_size);
      cv::cuda::GpuMat depthMask;
      depthMask.upload(cv::Mat::zeros(temp.size(), CV_8UC1));
      for (int i = 0; i < temp.rows; i++)
      {
         for (int j = 0; j < temp.cols; j++)
         {
            point_cloud2.getValue(0.5 * new_width - 150 + i, 0.5 * new_height - 150 + j, &point);
            float distance;
            distance = point;
            if (abs(distance / 100 - dist) < range)
            {
               depthMask.ptr<uchar>(j)[i] = 1;
            }
         }
      }
      return depthMask;
   }

   int getWidth()
   {
      return new_width;
   }
   int getHeight()
   {
      return new_height;
   }
};

#endif