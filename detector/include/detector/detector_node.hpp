#ifndef DETECTOR_NODE_HPP_
#define DETECTOR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include "detector/corner_detector.hpp"
#include "detector/pnp_solver.hpp" 
#include "interfaces/msg/center_delta.hpp"
#include "interfaces/msg/gimbal.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include <opencv2/opencv.hpp>
#include <mutex>
#include <atomic>
#include <cv_bridge/cv_bridge.h>


class DetectorNode : public rclcpp::Node
{
public:
  DetectorNode(const rclcpp::NodeOptions& options);
  ~DetectorNode();

private:
  std::unique_ptr<CornerDetector> CornerDetector_;

  //相机参数变量
  std::array<double, 9> camera_matrix_;
  std::vector<double> dist_coeffs_;
  double rectangle_width_;
  double rectangle_height_;

  //fps计算
  std::chrono::high_resolution_clock::time_point last_processing_time_;
  rclcpp::Time last_image_received_time_;
  double average_fps_ = 0.0;
  size_t frame_count_ = 0;

  float gimbal_pitch_ = 0.0f;
  float gimbal_yaw_ = 0.0f;
  
  // 发布器
  rclcpp::Publisher<interfaces::msg::CenterDelta>::SharedPtr center_delta_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr result_image_pub_; 
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr black_mask_pub_;
  
  // 订阅器
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<interfaces::msg::CenterDelta>::SharedPtr gimbal_feedback_sub_;

  // Camera info part
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
  cv::Point2f cam_center_;
  std::shared_ptr<sensor_msgs::msg::CameraInfo> cam_info_;
  std::unique_ptr<PnPSolver> pnp_solver_;
  
  // 心跳定时器
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void publishStatus();
  void publishMaskImage(const cv::Mat& mask, 
                       const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr& pub,
                       const std::string& frame_id);
  void publishResultImage(const cv::Mat& frame, 
                          const std::vector<cv::Point2f>& corners,
                          const cv::Point2f& image_center,
                          double transport_delay_ms,
                          double fps);
};

#endif  // DETECTOR_NODE_HPP_