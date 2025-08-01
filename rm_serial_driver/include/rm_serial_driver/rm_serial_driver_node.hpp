#ifndef SERIAL_DRIVER_MY_NODE_HPP_
#define SERIAL_DRIVER_MY_NODE_HPP_

#define READER_BUFFER_SIZE 64 // do not change this
#define MAX_BUFFER_SIZE 2048
#define DECODE_BUFFER_SIZE 128
#define TRANSMIT_BUFFER 16

// ROS
#include <rclcpp/logging.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/utilities.hpp>
#include <visualization_msgs/msg/marker.hpp>

// TF2
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// C++ system
#include <chrono>
#include <cstring>
#include <deque>
#include <functional>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <thread>
#include <vector>
#include <Eigen/Geometry>


#include "interfaces/msg/center_delta.hpp"
#include "interfaces/msg/gimbal.hpp"

#include "rm_serial_driver/crc.hpp"
#include "rm_serial_driver/packet.hpp"
#include "rm_serial_driver/serial_port.hpp"
// #include "rm_utils/math/utils.hpp"

namespace rm_serial_driver
{

  class SerialDriverNode : public rclcpp::Node
  {
  public:
    SerialDriverNode(const rclcpp::NodeOptions &options);

  private:
    // communicate with
    void rx();
    int receive();
    int transmit();
    void classify(uint8_t *data);
    PkgState decode();

    // communicate with RV
    // void autoAimDataCallback(auto_aim_interfaces::msg::Target::SharedPtr msg);
    void GimbalCmdCallback(interfaces::msg::CenterDelta::SharedPtr msg);
    // void ChassisCommandCallback(geometry_msgs::msg::Twist::SharedPtr msg);
    // void resetArmorTracker();
    // void setParam(const rclcpp::Parameter &param);
    void getParam();

    // uart
    std::shared_ptr<SerialConfig> config_;
    std::shared_ptr<Port> port_;
    std::string device_name_;

    std::deque<uint8_t> receive_buffer;
    std::deque<uint8_t> transmit_buffer;
    uint8_t decodeBuffer[DECODE_BUFFER_SIZE];
    uint8_t receiveBuffer[READER_BUFFER_SIZE];

    // protocol
    Header header;
    PkgState pkgState;

    std::mutex transmit_mutex;

    std::thread tx_thread;
    std::thread rx_thread;

    std::chrono::high_resolution_clock::time_point start =
    std::chrono::high_resolution_clock::now();
    std::string target_frame_;
    double timestamp_offset_ = 0;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // visualization_msgs::msg::Marker aiming_point_;

    // publish and subscribe
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr latency_pub_;
    rclcpp::Subscription<interfaces::msg::CenterDelta>::SharedPtr gimbal_cmd_sub_;
    rclcpp::Publisher<interfaces::msg::Gimbal>::SharedPtr gimbal_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr chassis_cmd_sub_;

    // Param client to set detect_color
    using ResultFuturePtr =
        std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>;

    bool initial_set_param_ = false;
    uint8_t previous_receive_color_ = 1; // 1 for BLUE, 0 for RED
    rclcpp::AsyncParametersClient::SharedPtr detector_param_client_;
    ResultFuturePtr set_param_future_;

    // Broadcast tf from odom to gimbal_link
    rclcpp::TimerBase::SharedPtr timer_transmit;

    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_tracker_client_;
    // rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

    // debug info
    bool crc_ok = false;
    bool crc_ok_header = false;

    int error_sum_payload = 0;
    int error_sum_header = 0;
    int read_sum = 0;
    int write_num = 0;
    int pkg_sum = 0;
    int classify_pkg_sum = 0;
    int trans_pkg_sum = 0;
    int state[5];
  };
} // namespace rm_serial_driver
#endif // SERIAL_DRIVER_MY_NODE_HPP_