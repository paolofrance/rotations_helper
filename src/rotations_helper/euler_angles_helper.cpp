#include <rotations_helper/euler_angles_helper.h>


static const char* green = "\033[32m";
static const char* yellow= "\033[33m";


EulerAnglesHelper::EulerAnglesHelper()
{
  init_angle_=false;
  previous_Z_angle_ = 0.0;
}

void EulerAnglesHelper::initAngle(const double& angle)
{
  previous_Z_angle_ = angle;
  init_angle_=true;
}


Eigen::Vector3d EulerAnglesHelper::getEulerAngles(const Eigen::Affine3d& matrix)
{
  if(!init_angle_)
    ROS_WARN("initial angle not initialized . using 0.0");
  
  return matrix.linear().eulerAngles(2, 1, 0).reverse();
}

Eigen::Vector3d EulerAnglesHelper::getEulerAnglesBounded(const Eigen::Affine3d& matrix)
{
  if(!init_angle_)
    ROS_WARN("initial angle not initialized . using 0.0");
  
  Eigen::Vector3d ret = getEulerAngles(matrix);
  
  ROS_DEBUG_STREAM("angle: " <<ret(2)<<", previous:"<<previous_Z_angle_<<", delta" <<ret(2)-previous_Z_angle_);
  
  if (ret(2)-previous_Z_angle_ > 0.1)
  {
    ret(2) -= M_PI ;
    ROS_INFO_STREAM_THROTTLE(1.0,green<<"bounding z: "<<ret(2));
  }
  else if (ret(2)-previous_Z_angle_ < -0.1)
  {      
    ret(2) += M_PI;
    ROS_INFO_STREAM_THROTTLE(1.0,yellow<<"bounding z: "<<ret(2));

  }
  previous_Z_angle_ = ret(2);
  
  return ret;
}
