#pragma once

#include <ros/ros.h>
#include <vector>
#include <eigen3/Eigen/Dense>
#include "utility/utility.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <fstream>

const double FOCAL_LENGTH = 460.0;
const int WINDOW_SIZE = 10;
const int NUM_OF_CAM = 1;                // 相机个数
const int NUM_OF_F = 1000;               // 特征数
//#define UNIT_SPHERE_ERROR

// extern 是声明变量，不是定义，声明不开辟内存。 这些变量是给到其它文件使用
// 关于extern 的资料： https://www.runoob.com/w3cnote/extern-head-h-different.html 
//                   https://blog.csdn.net/m0_46606290/article/details/119973574

extern double INIT_DEPTH;                // 深度初始值 
extern double MIN_PARALLAX;              // 关键帧选择阈值（像素单位）
extern int ESTIMATE_EXTRINSIC;           // IMU和相机的外参Rt:0准确；1不准确；2没有

extern double ACC_N, ACC_W;              // 加速度计噪声和随机偏置标准差
extern double GYR_N, GYR_W;              // 陀螺仪噪声和随机偏置标准差

extern std::vector<Eigen::Matrix3d> RIC; // 从相机到IMU的旋转矩阵
extern std::vector<Eigen::Vector3d> TIC; // 从相机到IMU的平移向量
extern Eigen::Vector3d G;                // 重力[0,0,g]

extern double BIAS_ACC_THRESHOLD;        // Ba阈值
extern double BIAS_GYR_THRESHOLD;        // Bg阈值
extern double SOLVER_TIME;               // 最大解算时间（以保证实时性）
extern int NUM_ITERATIONS;               // 最大解算器迭代次数（以保证实时性）
extern std::string EX_CALIB_RESULT_PATH; // 相机与IMU外参的输出路径OUTPUT_PATH + "/extrinsic_parameter.csv"
extern std::string VINS_RESULT_PATH;     // 输出路径OUTPUT_PATH + "/vins_result_no_loop.csv"
extern std::string IMU_TOPIC;
extern double TD;                        // IMU和cam的时间差. unit: s. readed image clock + td = real image clock (IMU clock)
extern double TR;                        // 卷帘快门每帧时间
extern int ESTIMATE_TD;                  // 在线校准IMU和camera时间
extern int ROLLING_SHUTTER;              // 1：卷帘快门相机；0：全局快门相机
extern double ROW, COL;                  // 图片的宽和高


void readParameters(ros::NodeHandle &n);

enum SIZE_PARAMETERIZATION
{
    SIZE_POSE = 7,
    SIZE_SPEEDBIAS = 9,
    SIZE_FEATURE = 1
};

// 状态变量在全体中的开始位置的索引，一般在协方差矩阵中会用到
enum StateOrder
{
    O_P = 0,
    O_R = 3,
    O_V = 6,
    O_BA = 9,
    O_BG = 12
};
// 噪声量的开始位置： 加计噪声、零偏噪声；陀螺仪噪声、零偏噪声
enum NoiseOrder
{
    O_AN = 0,
    O_GN = 3,
    O_AW = 6,
    O_GW = 9
};
