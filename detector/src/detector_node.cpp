#include "detector/detector_node.hpp"
#include <std_msgs/msg/string.hpp>
#include <chrono>

DetectorNode::DetectorNode(const rclcpp::NodeOptions & options)
: Node("detector_node", options),
  CornerDetector_(std::make_unique<CornerDetector>())
{
    // 创建发布器
    status_pub_ = this->create_publisher<std_msgs::msg::String>("/detector_status", 10);
    center_delta_pub_ = this->create_publisher<interfaces::msg::CenterDelta>("/center_delta", 10);
    result_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/result_img", 10);
    black_mask_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/black_img", 10);

    // 初始化时间点
    last_processing_time_ = std::chrono::high_resolution_clock::now();
    last_image_received_time_ = this->now();

    // 声明并获取矩形尺寸参数
    this->declare_parameter("rectangle_width", 0.279);
    this->declare_parameter("rectangle_height", 0.192);
    rectangle_width_ = this->get_parameter("rectangle_width").as_double();
    rectangle_height_ = this->get_parameter("rectangle_height").as_double();

    // 初始化云台角度
    gimbal_pitch_ = 0.0f;
    gimbal_yaw_ = 0.0f;

    // 初始化角点检测器
    CornerDetector_ = std::make_unique<CornerDetector>();

    cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera_info", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info) {
            cam_center_ = cv::Point2f(camera_info->k[2], camera_info->k[5]);
            cam_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(*camera_info);
            
            // 初始化PnP求解器
            pnp_solver_ = std::make_unique<PnPSolver>(
                camera_info->k, 
                camera_info->d,
                rectangle_width_, 
                rectangle_height_
            );
            cam_info_sub_.reset();  // 取消订阅
        });
    
    // 创建订阅器
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/image_raw", rclcpp::SensorDataQoS(),
        std::bind(&DetectorNode::imageCallback, this, std::placeholders::_1));

    // 创建云台姿态订阅器
    gimbal_feedback_sub_ = this->create_subscription<interfaces::msg::Gimbal>(
        "/gimbal", rclcpp::SensorDataQoS(),
        [this](interfaces::msg::Gimbal::SharedPtr msg) {
            // 更新云台角度
            gimbal_pitch_ = msg->gimbal_pitch;
            gimbal_yaw_ = msg->gimbal_yaw;
        });
    
    // 创建心跳定时器（独立于图像处理）
    heartbeat_timer_ = this->create_wall_timer(std::chrono::seconds(1),
        std::bind(&DetectorNode::publishStatus, this));
    
    RCLCPP_INFO(this->get_logger(), "节点初始化完成，等待图像数据...");
}

void DetectorNode::publishStatus()
{
    auto status_msg = std_msgs::msg::String();
    status_msg.data = "ALIVE";
    status_pub_->publish(status_msg);
}

void DetectorNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    try {
        auto received_time = this->now();
        auto processing_start = std::chrono::high_resolution_clock::now();
        
        // 计算FPS (使用指数平滑平均)
        auto current_time = std::chrono::high_resolution_clock::now();
        double duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            current_time - last_processing_time_).count();
            
        if (duration_ms > 0) {
        double instant_fps = 1000.0 / duration_ms;
        // 使用指数平滑计算平均FPS
        average_fps_ = (frame_count_ == 0) ? instant_fps : 0.7 * average_fps_ + 0.3 * instant_fps;
        frame_count_++;
        }
        last_processing_time_ = current_time;

        // 计算延迟 (从图像接收到处理开始)
        double transport_delay_ms = (received_time - last_image_received_time_).seconds() * 1000.0;
        last_image_received_time_ = received_time;
        
        // 转换ROS图像消息为OpenCV图像
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge异常: %s", e.what());
            return;
        }
        
        const cv::Mat& frame = cv_ptr->image;
        if (frame.empty()) {
            RCLCPP_WARN(this->get_logger(), "收到空图像帧");
            return;
        }
        
        cv::Point2f image_center(frame.cols / 2.0f, frame.rows / 2.0f);
        cv::Point2f box_center(0, 0);

        // 边框检测
        cv::Mat black_mask;
        std::vector<cv::Point2f> corners = CornerDetector_->Corner(frame, black_mask);
        
        if (corners.size() >= 5) {
            cv::Mat rvec, tvec;
            box_center.x = corners[4].x;
            box_center.y = corners[4].y;

            // 执行PnP解算
            if (pnp_solver_ && pnp_solver_->solveFixedRectanglePnP(
                {corners[0], corners[1], corners[2], corners[3]}, // 前4个角点
                rvec, tvec)) 
            {
                auto center_delta_msg = interfaces::msg::CenterDelta();

                center_delta_msg.red_position_x = box_center.x;
                center_delta_msg.red_position_y = box_center.y;
                center_delta_msg.purple_delta_x = box_center.x - image_center.x;
                center_delta_msg.purple_delta_y = box_center.y - image_center.y;

                if (center_delta_msg.purple_delta_x <= 20 && center_delta_msg.purple_delta_y <= 20)
                {
                    center_delta_msg.is_shoot = true;
                }
                
                // 添加3D位置信息（使用相同的前缀）
                center_delta_msg.box_center_x = tvec.at<double>(0);
                center_delta_msg.box_center_y = tvec.at<double>(1);
                center_delta_msg.box_center_z = tvec.at<double>(2);
                
                RCLCPP_INFO(this->get_logger(), "Publish center delta: (%.2f, %.2f, %.2f)",
                             center_delta_msg.box_center_x,
                             center_delta_msg.box_center_y,
                             center_delta_msg.box_center_z);

                // 获取平移向量分量
                double tx = tvec.at<double>(0);
                double ty = tvec.at<double>(1);
                double tz = tvec.at<double>(2);
                double yaw_rad = std::atan2(tx, tz);
                double pitch_rad = std::atan2(ty, tz);
                // 转换为角度（度）
                double yaw_deg = yaw_rad * 180.0 / CV_PI;
                double pitch_deg = pitch_rad * 180.0 / CV_PI;
            
                center_delta_msg.d_yaw = yaw_deg;
                center_delta_msg.d_pitch = pitch_deg;

                center_delta_msg.yaw = yaw_deg + gimbal_yaw_;
                center_delta_msg.pitch = pitch_deg + gimbal_pitch_;

                // RCLCPP_ERROR(this->get_logger(), "gimbal_yaw_%.2f°, gimbal_pitch_%.2f°", 
                //             gimbal_yaw_, gimbal_pitch_);

                center_delta_pub_->publish(center_delta_msg);

                RCLCPP_INFO(this->get_logger(), "Angles: Yaw=%.2f°, Pitch=%.2f°", 
                            yaw_deg, pitch_deg);
            }
        }

        // 发布掩膜图像
        if (!black_mask.empty()) {
            publishMaskImage(black_mask, black_mask_pub_, "black_mask");
        }

        // 在结果图像中显示
        publishResultImage(frame, corners, image_center, transport_delay_ms, average_fps_);
        
        // 计算处理延迟
        auto processing_end = std::chrono::high_resolution_clock::now();
        auto processing_duration = std::chrono::duration_cast<std::chrono::milliseconds>(
            processing_end - processing_start);
        
        RCLCPP_DEBUG(this->get_logger(), "处理时间: %.1f ms", 
                    static_cast<double>(processing_duration.count()));
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "处理错误: %s", e.what());
    }
}

void DetectorNode::publishMaskImage(const cv::Mat& mask, 
                                   const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr& pub,
                                   const std::string& frame_id)
{
    try {
        // 创建图像消息
        auto img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", mask).toImageMsg();
        img_msg->header.stamp = this->now();
        img_msg->header.frame_id = frame_id;
        
        // 发布图像
        pub->publish(*img_msg);
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "掩膜图像发布错误: %s", e.what());
    }
}

void DetectorNode::publishResultImage(const cv::Mat& frame, 
                                     const std::vector<cv::Point2f>& corners,
                                     const cv::Point2f& image_center,
                                     double transport_delay_ms,
                                     double fps)
{
    try {
        cv::Mat result_img = frame.clone();
        
        // 绘制延迟和FPS信息
        std::string delay_text = "Transport Delay: " + std::to_string(static_cast<int>(transport_delay_ms)) + "ms";
        std::string fps_text = "FPS: " + std::to_string(static_cast<int>(fps));
        
        cv::putText(result_img, delay_text, cv::Point(20, 40), 
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
        cv::putText(result_img, fps_text, cv::Point(20, 80), 
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);

        // 绘制连线与边框
        if (corners.size() >= 5) {
            // 仅当中心点有效时绘制
            cv::circle(result_img, image_center, 6, cv::Scalar(0, 255, 0), -1);
            
            // 仅当检测到有效中心点时绘制连线
            if (corners[4].x != 0 && corners[4].y != 0) {
                cv::line(result_img, image_center, corners[4], cv::Scalar(0, 255, 0), 2);
            }
        }

        if (corners.size() >= 4) {
            for (size_t i = 0; i < 4; i++) {
                cv::circle(result_img, corners[i], 4, cv::Scalar(255, 0, 0), 2);
                cv::putText(result_img, "O" + std::to_string(i+1), 
                        corners[i] + cv::Point2f(10, -10),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 1);
                if (i < 3) 
                    cv::line(result_img, corners[i], corners[i+1], cv::Scalar(255, 0, 0), 2);
                else
                    cv::line(result_img, corners[3], corners[0], cv::Scalar(255, 0, 0), 2);
            }
        }
        
        // 发布结果图像
        auto img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", result_img).toImageMsg();
        img_msg->header.stamp = this->now();
        img_msg->header.frame_id = "camera_frame";
        result_image_pub_->publish(*img_msg);
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "结果图像发布错误: %s", e.what());
    }
}

DetectorNode::~DetectorNode()
{
    RCLCPP_INFO(this->get_logger(), "节点安全关闭");
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(DetectorNode)