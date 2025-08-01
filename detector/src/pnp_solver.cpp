#include "detector/pnp_solver.hpp"
#include <opencv2/calib3d.hpp>

PnPSolver::PnPSolver(const std::array<double, 9> & camera_matrix,
                    const std::vector<double> & dist_coeffs,
                    double width, double height)  // 接收矩形尺寸参数
                    : camera_matrix_(cv::Mat(3, 3, CV_64F, const_cast<double*>(camera_matrix.data())).clone()),
                    dist_coeffs_(cv::Mat(1, dist_coeffs.size(), CV_64F, const_cast<double*>(dist_coeffs.data())).clone()),
                    rect_width_(width),
                    rect_height_(height)
{
  // 计算半宽高（转换为米）
  const double half_w = rect_width_ / 2.0;
  const double half_h = rect_height_ / 2.0;

  // 按照标准顺序创建3D模型点：左上、右上、右下、左下
  // 坐标系：X向前（垂直于平面），Y向左，Z向上
  rect_points_ = {
    cv::Point3f(0, -half_w, half_h),   // 左上 (Y负=左, Z正=上)
    cv::Point3f(0, half_w, half_h),    // 右上 (Y正=右, Z正=上)
    cv::Point3f(0, half_w, -half_h),   // 右下 (Y正=右, Z负=下)
    cv::Point3f(0, -half_w, -half_h)   // 左下 (Y负=左, Z负=下)
  };
}

// 新的PnP求解函数
bool PnPSolver::solveFixedRectanglePnP(
  const std::vector<cv::Point2f>& image_points,  // 输入点顺序：左上、右上、右下、左下
  cv::Mat & rvec,
  cv::Mat & tvec)
{
  // 验证输入点数量
  if (image_points.size() != 4) {
    return false;
  }

  // 直接使用预定义的3D模型点
  return cv::solvePnP(
    rect_points_,        // 3D模型点（固定顺序）
    image_points,        // 2D图像点（必须匹配顺序）
    camera_matrix_,      // 相机内参
    dist_coeffs_,        // 畸变系数
    rvec, tvec,          // 输出旋转和平移向量
    false,               // 不使用初始估计
    cv::SOLVEPNP_IPPE    // 使用适合平面物体的算法
  );
}
