#pragma once

#include <ros/ros.h>
#include <Eigen/Dense>
// #include <rotations_helper/utils.h>

class EulerAnglesHelper
{
public:

  EulerAnglesHelper();
  
  void initAngle(const double& angle);
  Eigen::Vector3d getEulerAngles(const Eigen::Affine3d& matrix);
  Eigen::Vector3d getEulerAnglesBounded(const Eigen::Affine3d& matrix);
  
protected:
  
  bool init_angle_;
  
  double previous_Z_angle_;
  
};

