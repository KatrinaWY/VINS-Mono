#pragma once

#include <ros/assert.h>
#include <ceres/ceres.h>
#include <Eigen/Dense>
#include "../utility/utility.h"
#include "../utility/tic_toc.h"
#include "../parameters.h"

// 它表示的是第i帧中单位相机平面上的点pts_i重投影到第j帧单位相机平面上的点与匹配点pts_j的投影误差
class ProjectionFactor : public ceres::SizedCostFunction<2, 7, 7, 7, 1>   // ceres::SizedCostFunction 是自定义代价函数的类
{
public:
  ProjectionFactor(const Eigen::Vector3d &_pts_i, const Eigen::Vector3d &_pts_j);
  virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
  void check(double **parameters);

  Eigen::Vector3d pts_i, pts_j;              // 角点在归一化坐标系下的3D坐标
  Eigen::Matrix<double, 2, 3> tangent_base;  // 角点在归一化平面的速度
  static Eigen::Matrix2d sqrt_info;          
  static double sum_t;
};
