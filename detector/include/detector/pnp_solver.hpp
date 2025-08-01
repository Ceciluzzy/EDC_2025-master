#ifndef DETECTOR__PNP_SOLVER_HPP_
#define DETECTOR__PNP_SOLVER_HPP_

#include <opencv2/core.hpp>
#include <vector>
#include <array>

class PnPSolver
{
public:
  // 添加矩形尺寸参数
  PnPSolver(
    const std::array<double, 9> & camera_matrix,
    const std::vector<double> & distortion_coefficients,
    double width, double height);

  // 修改为支持自定义点顺序的接口
  bool solveFixedRectanglePnP(
    const std::vector<cv::Point2f>& image_points,  // 输入点顺序：左上、右上、右下、左下
    cv::Mat & rvec,
    cv::Mat & tvec);

private:
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
  
  // 矩形尺寸（单位：米）
  double rect_width_;
  double rect_height_;
  
  // 矩形3D模型点（固定顺序）
  std::vector<cv::Point3f> rect_points_;
};


#endif  