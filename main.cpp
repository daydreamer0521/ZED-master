#include <iostream>

#include <opencv2/opencv.hpp>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "Zed.hpp"
 
using namespace std;
using namespace cv;
using namespace pcl;

// ctrl c中断
#include <signal.h>
bool ctrl_c_pressed = false;
void ctrlc(int)
{
    ctrl_c_pressed = true;
}

int main()
{
    // Ctrl C 中断
    signal(SIGINT, ctrlc);

    ZED Zed;
    Zed.init();
    Zed.setCamera();


    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Viewer_PCL"));

    // 设置默认的坐标系
    viewer->addCoordinateSystem(1.0);
    // 设置固定的元素。红色是X轴，绿色是Y轴，蓝色是Z
    viewer->addLine(pcl::PointXYZ(0, 0, 0), pcl::PointXYZ(10, 0, 0), "x");
    viewer->addLine(pcl::PointXYZ(0, 0, 0), pcl::PointXYZ(0, 5, 0), "y");
    viewer->addLine(pcl::PointXYZ(0, 0, 0), pcl::PointXYZ(0, 0, 2), "z");

    while (1)
    {
        if (ctrl_c_pressed == true)
            break;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        cv::Mat img;
        Zed.getCloud(cloud, img);

        timeval tt1, tt2;
        gettimeofday(&tt1, NULL);
        {
            Zed.update_sl_point_cloud();
            for (int i = -10; i <= 10; i++)
            {
                for (int j = -10; j <= 10; j++)
                {
                    float world3[3];
                    Zed.coordinate_from_sl_point_cloud(Point(img.size().width / 2 + i, img.size().height / 2 + j), world3);
                    // printf("%f %f %f\n", world3[0], world3[1], world3[2]);
                }
            }
        }
        gettimeofday(&tt2, NULL);
        printf("time: %f\n", (tt2.tv_sec - tt1.tv_sec) * 1000.0 + (tt2.tv_usec - tt1.tv_usec) / 1000.0);

        Zed.calcDistance(Point(img.size().width / 2, img.size().height / 2));

        imshow("img", img);
        // waitKey(3);

        viewer->addPointCloud(cloud, "cloud");
        viewer->spinOnce(3);
        viewer->removePointCloud("cloud");

        if (cv::waitKey(3) == 27) break;
    }

        // 程序退出前保存最后一帧点云
    if (!viewer->wasStopped())
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        cv::Mat final_img;
        Zed.getCloud(final_cloud, final_img);
        
        if (!final_cloud->empty())
        {
            pcl::io::savePCDFileBinary("final_pointcloud.pcd", *final_cloud);
            cv::imwrite("final_image.png", final_img);
            std::cout << "最终点云和图像已保存" << std::endl;
        }
    }



    Zed.close();
}