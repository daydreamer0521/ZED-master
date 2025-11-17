#!/usr/bin/python3
# >>>>>>
# Descripttion:
# version: 1.0
# Author: Zx
# Email: ureinsecure@outlook.com
# Date: 2021-05-08 19:49:54
# LastEditors: Zx
# LastEditTime: 2022-05-01 20:42:36
# FilePath: /Jetson_IR/set_config.py
# <<<<<<
import cv2
import numpy as np
import platform

CONFIG_PATH = "Zed_Settings.yaml"

print(CONFIG_PATH)

f = cv2.FileStorage(CONFIG_PATH, cv2.FileStorage_WRITE)

v = {}
v["brightness"] = 4 # 图片亮度
v["contrast"] = 4   # 对比度
v["hue"] = 0        # 色调
v["saturation"] = 4 # 饱和度
v["sharpness"] = 5  # 锐度
v["gamma"] = 0      # 伽马矫正
v["gain"] = -1      # 曝光增益
v["exposure"] = -1  # 曝光时间

head = ""
src = ""
for key, value in v.items():
    f.write(key, value)
    head += ("{} {} = {};\n".format(str(type(value)).
                                    replace("<class '", "").replace("'>", "").replace("float", "double"), key, value))
    src += ('f["{0}"] >> {0};\n'.format(key))

f.releaseAndGetString()
print("// >>>>>> HEAD")
print(head, end="")
print("// <<<<<< HEAD")
print("")

print("// >>>>>> SRC")
print(src, end="")
print("// <<<<<< SRC")
print("")

print("set config done, config path is {}".format(CONFIG_PATH))

# example: https://github.com/innns/junkcar
"""
cv::FileStorage f(CONFIG_PATH, cv::FileStorage::READ);
if (f.isOpened())
{
    cout << "Using config\n";

    f["res_w"] >> res_w;
    f["res_h"] >> res_h;

    Mat t; //temp

    f["HSV_R"] >> t;
    HSV_R.min = Scalar(t.at<double>(0, 0), t.at<double>(0, 1), t.at<double>(0, 2));
    HSV_R.max = Scalar(t.at<double>(1, 0), t.at<double>(1, 1), t.at<double>(1, 2));

    f["HC"] >> t;
    HC.dp = t.at<double>(0);
    HC.minDist = t.at<double>(1);
    HC.param1 = t.at<double>(2);
    HC.param2 = t.at<double>(3);
    HC.minRadius = (int)t.at<double>(4);
    HC.maxRadius = (int)t.at<double>(5);

    f["CEN_L"] >> t;
    CEN_L.x = t.at<double>(0);
    CEN_L.y = t.at<double>(1);

    f["IP"] >> IP;
    f["PORT"] >> PORT;
}
else
{
    cout << "Using default config\n";
}
f.release();
"""