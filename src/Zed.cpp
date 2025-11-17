#include "Zed.hpp"

#include <Zed_cuda.h>

ZED::ZED()
{
    cv::FileStorage file("../yaml/Zed_Settings.yaml", cv::FileStorage::READ);

    if (file.isOpened())
    {
        time_t tt = time(NULL);
        tm *pTime = localtime(&tt);
        int nHour = pTime->tm_hour;
        // std::cout<<"hour: "<<nHour<<endl;
        if (nHour >= 6 && nHour <= 12)
        {
            file["front_brightness_day"] >> ZED::yaml_zed_settings[0].brightness;                             // 亮度
            file["front_contrast_day"] >> ZED::yaml_zed_settings[0].contrast;                                 // 对比度
            file["front_hue_day"] >> ZED::yaml_zed_settings[0].hue;                                           // 色调
            file["front_saturation_day"] >> ZED::yaml_zed_settings[0].saturation;                             // 饱和度
            file["front_sharpness_day"] >> ZED::yaml_zed_settings[0].sharpness;                               // 锐度
            file["front_gamma_day"] >> ZED::yaml_zed_settings[0].gamma;                                       // 伽马矫正
            file["front_gain_day"] >> ZED::yaml_zed_settings[0].gain;                                         // 曝光增益
            file["front_exposure_day"] >> ZED::yaml_zed_settings[0].exposure;                                 // 曝光时间
            file["front_whitebalance_temperature_day"] >> ZED::yaml_zed_settings[0].whitebalance_temperature; // 白平衡

            file["behind_brightness_day"] >> ZED::yaml_zed_settings[1].brightness;                             // 亮度
            file["behind_contrast_day"] >> ZED::yaml_zed_settings[1].contrast;                                 // 对比度
            file["behind_hue_day"] >> ZED::yaml_zed_settings[1].hue;                                           // 色调
            file["behind_saturation_day"] >> ZED::yaml_zed_settings[1].saturation;                             // 饱和度
            file["behind_sharpness_day"] >> ZED::yaml_zed_settings[1].sharpness;                               // 锐度
            file["behind_gamma_day"] >> ZED::yaml_zed_settings[1].gamma;                                       // 伽马矫正
            file["behind_gain_day"] >> ZED::yaml_zed_settings[1].gain;                                         // 曝光增益
            file["behind_exposure_day"] >> ZED::yaml_zed_settings[1].exposure;                                 // 曝光时间
            file["behind_whitebalance_temperature_day"] >> ZED::yaml_zed_settings[1].whitebalance_temperature; // 白平衡
        }
        else if (nHour > 12 && nHour <= 18)
        {
            file["front_brightness_noon"] >> ZED::yaml_zed_settings[0].brightness;                             // 亮度
            file["front_contrast_noon"] >> ZED::yaml_zed_settings[0].contrast;                                 // 对比度
            file["front_hue_noon"] >> ZED::yaml_zed_settings[0].hue;                                           // 色调
            file["front_saturation_noon"] >> ZED::yaml_zed_settings[0].saturation;                             // 饱和度
            file["front_sharpness_noon"] >> ZED::yaml_zed_settings[0].sharpness;                               // 锐度
            file["front_gamma_noon"] >> ZED::yaml_zed_settings[0].gamma;                                       // 伽马矫正
            file["front_gain_noon"] >> ZED::yaml_zed_settings[0].gain;                                         // 曝光增益
            file["front_exposure_noon"] >> ZED::yaml_zed_settings[0].exposure;                                 // 曝光时间
            file["front_whitebalance_temperature_noon"] >> ZED::yaml_zed_settings[0].whitebalance_temperature; // 白平衡

            file["behind_brightness_noon"] >> ZED::yaml_zed_settings[1].brightness;                             // 亮度
            file["behind_contrast_noon"] >> ZED::yaml_zed_settings[1].contrast;                                 // 对比度
            file["behind_hue_noon"] >> ZED::yaml_zed_settings[1].hue;                                           // 色调
            file["behind_saturation_noon"] >> ZED::yaml_zed_settings[1].saturation;                             // 饱和度
            file["behind_sharpness_noon"] >> ZED::yaml_zed_settings[1].sharpness;                               // 锐度
            file["behind_gamma_noon"] >> ZED::yaml_zed_settings[1].gamma;                                       // 伽马矫正
            file["behind_gain_noon"] >> ZED::yaml_zed_settings[1].gain;                                         // 曝光增益
            file["behind_exposure_noon"] >> ZED::yaml_zed_settings[1].exposure;                                 // 曝光时间
            file["behind_whitebalance_temperature_noon"] >> ZED::yaml_zed_settings[1].whitebalance_temperature; // 白平衡
        }
        else
        {
            file["front_brightness_night"] >> ZED::yaml_zed_settings[0].brightness;                             // 亮度
            file["front_contrast_night"] >> ZED::yaml_zed_settings[0].contrast;                                 // 对比度
            file["front_hue_night"] >> ZED::yaml_zed_settings[0].hue;                                           // 色调
            file["front_saturation_night"] >> ZED::yaml_zed_settings[0].saturation;                             // 饱和度
            file["front_sharpness_night"] >> ZED::yaml_zed_settings[0].sharpness;                               // 锐度
            file["front_gamma_night"] >> ZED::yaml_zed_settings[0].gamma;                                       // 伽马矫正
            file["front_gain_night"] >> ZED::yaml_zed_settings[0].gain;                                         // 曝光增益
            file["front_exposure_night"] >> ZED::yaml_zed_settings[0].exposure;                                 // 曝光时间
            file["front_whitebalance_temperature_night"] >> ZED::yaml_zed_settings[0].whitebalance_temperature; // 白平衡

            file["behind_brightness_night"] >> ZED::yaml_zed_settings[1].brightness;                             // 亮度
            file["behind_contrast_night"] >> ZED::yaml_zed_settings[1].contrast;                                 // 对比度
            file["behind_hue_night"] >> ZED::yaml_zed_settings[1].hue;                                           // 色调
            file["behind_saturation_night"] >> ZED::yaml_zed_settings[1].saturation;                             // 饱和度
            file["behind_sharpness_night"] >> ZED::yaml_zed_settings[1].sharpness;                               // 锐度
            file["behind_gamma_night"] >> ZED::yaml_zed_settings[1].gamma;                                       // 伽马矫正
            file["behind_gain_night"] >> ZED::yaml_zed_settings[1].gain;                                         // 曝光增益
            file["behind_exposure_night"] >> ZED::yaml_zed_settings[1].exposure;                                 // 曝光时间
            file["behind_whitebalance_temperature_night"] >> ZED::yaml_zed_settings[1].whitebalance_temperature; // 白平衡
        }
    }

    file.release();
}

/**
 * 初始化zed相机
 */
void ZED::init()
{
    // 设置参数
    sl::InitParameters init_params;
    init_params.camera_resolution = sl::RESOLUTION::HD1080;

    init_params.camera_fps = 30;

    init_params.coordinate_units = sl::UNIT::METER;
    init_params.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP;
    init_params.depth_mode = sl::DEPTH_MODE::ULTRA;

    init_params.depth_minimum_distance = 0.5;
    init_params.depth_maximum_distance = 6;

    // 打开相机
    sl::ERROR_CODE returned_state = zed.open(init_params);
    if (returned_state != sl::ERROR_CODE::SUCCESS)
    {
        std::cout << "Error " << returned_state << ", exit program.\n";
        exit(1);
    }

    // 设置runtime参数
    runtime_params = sl::RuntimeParameters();
    // runtime_params.sensing_mode = sl::SENSING_MODE::STANDARD;
    // 设置点云置信度
    runtime_params.confidence_threshold = 100;

    image_size = zed.getCameraInformation().camera_configuration.resolution;
    new_width = image_size.width;
    new_height = image_size.height;

    ZED::setCamera();
}
void ZED::init(size_t ZED_SN)
{
    // 设置参数
    sl::InitParameters init_params;
    init_params.camera_resolution = sl::RESOLUTION::HD1080;

    init_params.camera_fps = 30;

    init_params.coordinate_units = sl::UNIT::METER;
    init_params.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP;
    init_params.depth_mode = sl::DEPTH_MODE::ULTRA;

    init_params.depth_minimum_distance = 0.5;
    init_params.depth_maximum_distance = 6;

    init_params.input.setFromSerialNumber(ZED_SN);

    // 打开相机
    int zed_num = 0;
    if (ZED_SN == ZED_SN_0)
        zed_num = 0;
    if (ZED_SN == ZED_SN_1)
        zed_num = 1;

    sl::ERROR_CODE err = zeds[zed_num].open(init_params);

    if (err == sl::ERROR_CODE::SUCCESS)
    {
        zeds_info[zed_num] = zeds[zed_num].getCameraInformation();
        std::cout << "ZED " << zeds_info[zed_num].camera_model << ", SN: " << zeds_info[zed_num].serial_number << " Opened" << endl;
    }
    else
    {
        std::cout << "Error " << err << ", exit program.\n";
    }

    // 设置runtime参数
    runtime_params = sl::RuntimeParameters();
    // runtime_params.sensing_mode = sl::SENSING_MODE::STANDARD;
    // 设置点云置信度
    runtime_params.confidence_threshold = 100;

    image_size = zeds[zed_num].getCameraInformation().camera_configuration.resolution;
    new_width = image_size.width;
    new_height = image_size.height;

    ZED::setCamera(ZED_SN);
}

/**
 * 设置zed相机参数
 */
void ZED::setCamera()
{
    zed.setCameraSettings(sl::VIDEO_SETTINGS::BRIGHTNESS, yaml_zed_settings[0].brightness);                             // 图片亮度
    zed.setCameraSettings(sl::VIDEO_SETTINGS::CONTRAST, yaml_zed_settings[0].contrast);                                 // 对比度
    zed.setCameraSettings(sl::VIDEO_SETTINGS::HUE, yaml_zed_settings[0].hue);                                           // 色调
    zed.setCameraSettings(sl::VIDEO_SETTINGS::SATURATION, yaml_zed_settings[0].saturation);                             // 饱和度
    zed.setCameraSettings(sl::VIDEO_SETTINGS::SHARPNESS, yaml_zed_settings[0].sharpness);                               // 锐度
    zed.setCameraSettings(sl::VIDEO_SETTINGS::GAMMA, yaml_zed_settings[0].gamma);                                       // 伽马矫正
    zed.setCameraSettings(sl::VIDEO_SETTINGS::WHITEBALANCE_TEMPERATURE, yaml_zed_settings[0].whitebalance_temperature); // 自动白平衡
    zed.setCameraSettings(sl::VIDEO_SETTINGS::GAIN, yaml_zed_settings[0].gain);                                         // 亮度增益
    zed.setCameraSettings(sl::VIDEO_SETTINGS::EXPOSURE, yaml_zed_settings[0].exposure);                                 // 曝光时间
}
/**
 * 设置指定SN码zed相机参数
 * @param serial_number 指定SN码
 */
void ZED::setCamera(unsigned int serial_number)
{
    // 获取对应的数组下标
    size_t SN_number = 0;
    for (SN_number = 0; SN_number < size_devList; SN_number++)
        if (zeds_info[SN_number].serial_number == serial_number)
            break;

    zeds[SN_number].setCameraSettings(sl::VIDEO_SETTINGS::BRIGHTNESS, yaml_zed_settings[SN_number].brightness);                             // 图片亮度
    zeds[SN_number].setCameraSettings(sl::VIDEO_SETTINGS::CONTRAST, yaml_zed_settings[SN_number].contrast);                                 // 对比度
    zeds[SN_number].setCameraSettings(sl::VIDEO_SETTINGS::HUE, yaml_zed_settings[SN_number].hue);                                           // 色调
    zeds[SN_number].setCameraSettings(sl::VIDEO_SETTINGS::SATURATION, yaml_zed_settings[SN_number].saturation);                             // 饱和度
    zeds[SN_number].setCameraSettings(sl::VIDEO_SETTINGS::SHARPNESS, yaml_zed_settings[SN_number].sharpness);                               // 锐度
    zeds[SN_number].setCameraSettings(sl::VIDEO_SETTINGS::GAMMA, yaml_zed_settings[SN_number].gamma);                                       // 伽马矫正
    zeds[SN_number].setCameraSettings(sl::VIDEO_SETTINGS::WHITEBALANCE_TEMPERATURE, yaml_zed_settings[SN_number].whitebalance_temperature); // 自动白平衡
    zeds[SN_number].setCameraSettings(sl::VIDEO_SETTINGS::GAIN, yaml_zed_settings[SN_number].gain);                                         // 亮度增益
    zeds[SN_number].setCameraSettings(sl::VIDEO_SETTINGS::EXPOSURE, yaml_zed_settings[SN_number].exposure);                                 // 曝光时间
    // zeds[SN_number].setCameraSettings(sl::VIDEO_SETTINGS::GAIN, -1);        // 亮度增益
    // zeds[SN_number].setCameraSettings(sl::VIDEO_SETTINGS::EXPOSURE, -1);    // 曝光时间

    // printf("whitebalance temperature %d\n",zeds[SN_number].getCameraSettings(sl::VIDEO_SETTINGS::WHITEBALANCE_TEMPERATURE));
}

/**
 * 获取zed相机图像
 * @return vector<cv::Mat> pictures 表示获取到的图像； pictures[0] 彩色图像； pictures[1] 深度图像；
 */
vector<cv::Mat> ZED::getImg()
{
    sl::Mat depth_image_zed(new_width, new_height, sl::MAT_TYPE::U8_C4, sl::MEM::GPU);
    sl::Mat image_zed(new_width, new_height, sl::MAT_TYPE::U8_C4);
    if (zed.grab(runtime_params) == sl::ERROR_CODE::SUCCESS)
    {
        zed.retrieveImage(depth_image_zed, sl::VIEW::DEPTH, sl::MEM::GPU, image_size);
        zed.retrieveImage(image_zed, sl::VIEW::LEFT, sl::MEM::CPU, image_size);
    }
    cv::Mat image_ocv = slMat2cvMat(image_zed);
    cv::cuda::GpuMat depth_image_ocv = slMat2cvMatGPU(depth_image_zed);
    cv::Mat depth_image_ocv_m;
    depth_image_ocv.download(depth_image_ocv_m);
    vector<cv::Mat> pictures;
    pictures.push_back(image_ocv);
    pictures.push_back(depth_image_ocv_m);
    return pictures;
}

/**
 *  This function convert a RGBA color packed into a packed RGBA PCL compatible format
 **/
inline float convertColor(float colorIn)
{
    uint32_t color_uint = *(uint32_t *)&colorIn;
    unsigned char *color_uchar = (unsigned char *)&color_uint;
    color_uint = ((uint32_t)color_uchar[0] << 16 | (uint32_t)color_uchar[1] << 8 | (uint32_t)color_uchar[2]);
    return *reinterpret_cast<float *>(&color_uint);
}

/**
 * 获取点云
 * @return 获取到的点云
 */
void ZED::getCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr vis_cloud_return, cv::Mat &image_return)
{
    // 点云初始化
    sl::Mat data_cloud;
    sl::Resolution cloud_res;
    // cloud_res = sl::Resolution(1280, 720);
    // cloud_res = sl::Resolution(960, 540);
    cloud_res = sl::Resolution(640, 360);
    // cloud_res = sl::Resolution(320, 180);
    size_t cloud_size = cloud_res.area();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_pcl_point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    p_pcl_point_cloud->points.resize(cloud_size);

    sl::Mat image_zed(new_width, new_height, sl::MAT_TYPE::U8_C4);
    if (zed.grab(runtime_params) == sl::ERROR_CODE::SUCCESS)
    {
        zed.retrieveMeasure(data_cloud, sl::MEASURE::XYZRGBA, sl::MEM::CPU, cloud_res);
        zed.retrieveImage(image_zed, sl::VIEW::LEFT, sl::MEM::CPU, image_size);
    }
    else
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr zero_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        cout << "[ERROR] zed cloud capture error!" << endl;
        // return zero_cloud;
        return;
    }
    cv::Mat image_ocv = slMat2cvMat(image_zed);

    float *p_data_cloud = data_cloud.getPtr<float>();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr vis_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    vis_cloud->points.resize(cloud_size);

    // cuda处理点云
    processPointCloud(p_data_cloud, cloud_size, vis_cloud);

    vis_cloud->width = vis_cloud->size();
    vis_cloud->height = 1;
    vis_cloud->is_dense = false;
    // std::cout << vis_cloud->size() << std::endl;

    // return vis_cloud;
    pcl::copyPointCloud(*vis_cloud, *vis_cloud_return);
    image_return = image_ocv.clone();
}
/**
 * 获取指定SN码zed点云
 * @return 获取到的点云
 */
void ZED::getCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr vis_cloud_return, cv::Mat &image_return, unsigned int serial_number)
{
    // 获取对应的数组下标
    size_t SN_number = 0;
    for (SN_number = 0; SN_number < size_devList; SN_number++)
        if (zeds_info[SN_number].serial_number == serial_number)
            break;

    // 点云初始化
    sl::Mat data_cloud;
    sl::Resolution cloud_res;
    // cloud_res = sl::Resolution(1280, 720);
    // cloud_res = sl::Resolution(960, 540);
    cloud_res = sl::Resolution(640, 360);
    // cloud_res = sl::Resolution(320, 180);
    size_t cloud_size = cloud_res.area();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_pcl_point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    p_pcl_point_cloud->points.resize(cloud_size);

    sl::Mat image_zed(new_width, new_height, sl::MAT_TYPE::U8_C4);
    if (zeds[SN_number].grab(runtime_params) == sl::ERROR_CODE::SUCCESS)
    {
        zeds[SN_number].retrieveMeasure(data_cloud, sl::MEASURE::XYZRGBA, sl::MEM::CPU, cloud_res);
        zeds[SN_number].retrieveImage(image_zed, sl::VIEW::LEFT, sl::MEM::CPU, image_size);
    }
    else
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr zero_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        // return zero_cloud;
        return;
    }
    cv::Mat image_ocv = slMat2cvMat(image_zed);

    float *p_data_cloud = data_cloud.getPtr<float>();
    int index = 0;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr vis_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    vis_cloud->points.resize(cloud_size);

    // cuda处理点云
    processPointCloud(p_data_cloud, cloud_size, vis_cloud);

    vis_cloud->width = vis_cloud->size();
    vis_cloud->height = 1;
    vis_cloud->is_dense = false;
    // std::cout << vis_cloud->size() << std::endl;

    // return vis_cloud;
    pcl::copyPointCloud(*vis_cloud, *vis_cloud_return);
    image_return = image_ocv.clone();
}

/**
 * 获取像素点的深度
 * @param center 像素点的坐标
 * @return 像素点的深度
 */
float ZED::calcDistance(cv::Point center)
{
    sl::Mat point_cloud;
    // zed.retrieveImage(image, VIEW::LEFT);
    zed.retrieveMeasure(point_cloud, sl::MEASURE::DEPTH, sl::MEM::CPU, image_size);
    sl::float1 point3D;
    point_cloud.getValue(center.x, center.y, &point3D);
    float distance = point3D;
    return distance;
}

/**
 * 从像素坐标转为世界坐标
 * @param center 像素坐标
 * @param Wolrd 世界坐标； World[0] 世界坐标x； World[1] 世界坐标y； World[2] 世界坐标z；
 */
void ZED::coordinate(cv::Point center, float *World)
{
    sl::Mat point_cloud;
    // zed.retrieveImage(image, VIEW::LEFT);
    //  zed.retrieveMeasure(point_cloud, sl::MEASURE::DEPTH, sl::MEM::CPU, image_size);
    zed.retrieveMeasure(point_cloud, sl::MEASURE::XYZRGBA, sl::MEM::CPU, image_size); // Get the point cloud
    sl::float4 point3D;
    point_cloud.getValue(center.x, center.y, &point3D);

    World[0] = point3D.x, World[1] = point3D.y, World[2] = point3D.z;

    return;
}

/**
 * @brief 更新sl点云，并存储在private中
 *
 */
void ZED::update_sl_point_cloud()
{
    zed.retrieveMeasure(sl_point_cloud, sl::MEASURE::XYZRGBA, sl::MEM::CPU, image_size); // Get the point cloud
}

/**
 * @brief 从sl点云中获取世界坐标
 *
 * @param center 像素坐标
 * @param World 世界坐标； World[0] 世界坐标x； World[1] 世界坐标y； World[2] 世界坐标z；
 */
void ZED::coordinate_from_sl_point_cloud(cv::Point center, float *World)
{
    sl::float4 point3D;
    sl_point_cloud.getValue(center.x, center.y, &point3D);

    World[0] = point3D.x, World[1] = point3D.y, World[2] = point3D.z;

    return;
}

/**
 * 获取opencv图像类型
 * @param type sl图像类型
 * @return opencv图像类型
 */
int ZED::getOCVtype(sl::MAT_TYPE type)
{
    int cv_type = -1;
    switch (type)
    {
    case sl::MAT_TYPE::F32_C1:
        cv_type = CV_32FC1;
        break;
    case sl::MAT_TYPE::F32_C2:
        cv_type = CV_32FC2;
        break;
    case sl::MAT_TYPE::F32_C3:
        cv_type = CV_32FC3;
        break;
    case sl::MAT_TYPE::F32_C4:
        cv_type = CV_32FC4;
        break;
    case sl::MAT_TYPE::U8_C1:
        cv_type = CV_8UC1;
        break;
    case sl::MAT_TYPE::U8_C2:
        cv_type = CV_8UC2;
        break;
    case sl::MAT_TYPE::U8_C3:
        cv_type = CV_8UC3;
        break;
    case sl::MAT_TYPE::U8_C4:
        cv_type = CV_8UC4;
        break;
    default:
        break;
    }
    return cv_type;
}

/**
 * 获取opencv图像
 * @param input sl图像
 * @return opencv图像
 */
cv::Mat ZED::slMat2cvMat(sl::Mat &input)
{
    // Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
    // cv::Mat and sl::Mat will share a single memory structure
    cv::Mat res_image = cv::Mat(input.getHeight(), input.getWidth(), getOCVtype(input.getDataType()), input.getPtr<sl::uchar1>(sl::MEM::CPU), input.getStepBytes(sl::MEM::CPU));
    cv::cvtColor(res_image, res_image, COLOR_RGBA2RGB); // 从四通道转到三通道
    return res_image;
}

/**
 * 获取opencv GpuMat图像
 * @param input sl图像
 * @return opencv GpuMat图像
 */
cv::cuda::GpuMat ZED::slMat2cvMatGPU(sl::Mat &input)
{
    // Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
    // cv::Mat and sl::Mat will share a single memory structure
    return cv::cuda::GpuMat(input.getHeight(), input.getWidth(), getOCVtype(input.getDataType()), input.getPtr<sl::uchar1>(sl::MEM::GPU), input.getStepBytes(sl::MEM::GPU));
}

// 关闭zed相机
void ZED::close()
{
    zed.close();
}
void ZED::close(size_t ZED_SN)
{
    // 获取对应的数组下标
    size_t SN_number = 0;
    for (SN_number = 0; SN_number < size_devList; SN_number++)
        if (zeds_info[SN_number].serial_number == ZED_SN)
            break;
    zeds[SN_number].close();
    std::cout << "ZED " << zeds_info[SN_number].camera_model << ", SN: " << zeds_info[SN_number].serial_number << " Closed" << endl;
}