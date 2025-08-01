#include "rm_serial_driver/rm_serial_driver_node.hpp"
#include <Eigen/Geometry>
#include <tf2/LinearMath/Matrix3x3.h>
// std
#include <chrono>
#include <cstdint>
#include <memory>
#include <thread>
// ros2
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
// project
#define DEBUG_SERIAL_DRIVER 0
namespace rm_serial_driver
{
  SerialDriverNode::SerialDriverNode(const rclcpp::NodeOptions &options)
      : rclcpp::Node("rm_serial_driver_node", options)
  {
    getParam();
    //创建串口对象
    port_ = std::make_shared<Port>(config_);
    RCLCPP_INFO(get_logger(), "Begin the driver Node !");

    //声明时间戳偏移参数，默认为 0.0
    timestamp_offset_ = this->declare_parameter("timestamp_offset", 0.0);

    detector_param_client_ =
        std::make_shared<rclcpp::AsyncParametersClient>(this, "detector");

    gimbal_pub_ = this->create_publisher<interfaces::msg::Gimbal>("/gimbal", 10);

    latency_pub_ =
        this->create_publisher<std_msgs::msg::Float64>("/latency", 10);

    for (int i = 0; i < 4; i++)
    {
      if (port_->isPortOpen())
      {
        break;
      }
      else
      {
        port_->openPort();
      }
    }
    if (!port_->isPortOpen())
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to open port");
      return;
    }

    gimbal_cmd_sub_ = this->create_subscription<interfaces::msg::CenterDelta>(
        "center_delta", rclcpp::SensorDataQoS(),
        std::bind(&SerialDriverNode::GimbalCmdCallback, this, std::placeholders::_1));

    // 创建定时器用于数据传输
    timer_transmit =
        this->create_wall_timer(std::chrono::milliseconds(10),
                                std::bind(&SerialDriverNode::transmit, this));

    // 启动接收线程
    rx_thread = std::thread(&SerialDriverNode::rx, this);
  }

  // 接收线程函数
  void SerialDriverNode::rx()
  {
    while (rclcpp::ok()) 
    {
      receive();
      pkgState = PkgState::COMPLETE;
      while (receive_buffer.size() > 0 &&
             pkgState != PkgState::HEADER_INCOMPLETE &&
             pkgState != PkgState::PAYLOAD_INCOMPLETE)
      {
        pkgState = decode();
      }
    }
  }
  
  // 接收数据函数
  int SerialDriverNode::receive()
  {
    int read_num = 0;

    // read_num =read(port_->fd,receiveBuffer, 64);
    read_num = port_->receive(receiveBuffer);
    read_sum += read_num;
    if (read_num > 0)
    {
      receive_buffer.insert(receive_buffer.end(), receiveBuffer,
                            receiveBuffer + read_num);
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "Can not read from the uart !");
      port_->closePort();
      rclcpp::sleep_for(std::chrono::seconds(1));
      port_->openPort();
    }
    return read_num;
  }


  int SerialDriverNode::transmit()
  {
    // 创建临时缓冲区，大小为 TRANSMIT_BUFFER
    uint8_t buffer[TRANSMIT_BUFFER];
    // 获取当前传输缓冲区中的数据量
    long size = transmit_buffer.size();


    // 检查缓冲区是否有数据
    if (size)
    {
        // 当数据量大于2倍传输缓冲区大小且缓冲区不为空时循环
        while (size > 2 * TRANSMIT_BUFFER && transmit_buffer.size() > 0)
        {
            // 更新剩余数据量
            size -= TRANSMIT_BUFFER;
            // 加锁保护传输缓冲区（防止多线程同时访问）
            std::lock_guard<std::mutex> lockf(transmit_mutex);
            // 从传输缓冲区复制数据到临时缓冲区
            std::copy(transmit_buffer.begin(),
                      transmit_buffer.begin() + TRANSMIT_BUFFER, 
                      buffer);
            // 从传输缓冲区中移除已复制的数据
            transmit_buffer.erase(transmit_buffer.begin(),
                                 transmit_buffer.begin() + TRANSMIT_BUFFER);
            // 通过串口对象发送数据
            write_num = port_->transmit(buffer, TRANSMIT_BUFFER);
            // 错误处理：发送失败时
            if (write_num < 0)
            {
                RCLCPP_ERROR(get_logger(), "Can not transmit");
                port_->closePort();
                rclcpp::sleep_for(std::chrono::seconds(1));
                port_->openPort();
            }
        // GimbalCommand pack = bufferToStruct<GimbalCommand>(buffer);
        // for(int i = 0; i < 33; i++){
        //   printf("%02X ", buffer[i]);
        // }
        // printf("\n");
        // RCLCPP_INFO(get_logger(), "transmit: pitch:%f yaw:%f is_shoot:%d", pack.pitch, pack.yaw, pack.is_shoot);
      }
    }
    return write_num;
  }

  // void SerialDriverNode::classify(uint8_t *data)
  // {
  //   // Header header = arrayToStruct<Header>(data);
  //   // target_frame_ = this->declare_parameter("target_frame","odom");
  //   //解析云台消息
  //   InfantryGimbalMsg packet = bufferToStruct<InfantryGimbalMsg>(data);
  //   if (packet.header.protocolID != INFANTRY_GIMBAL_MSG)
  //     return;
  //   try
  //   {
  //     RCLCPP_INFO(get_logger(), "classify: pitch:%f yaw:%f", packet.pitch, packet.yaw);
  //     RCLCPP_INFO(get_logger(), "data %d ", data);
  //     classify_pkg_sum++;
  //     double pitch, yaw;
  //     pitch = packet.pitch * M_PI / 180.0;
  //     yaw = packet.yaw * M_PI / 180.0;
  //     auto gimbal_msg = interfaces::msg::Gimbal();
  //     gimbal_msg.gimbal_pitch = pitch;
  //     gimbal_msg.gimbal_yaw = yaw;
      
  //     gimbal_pub_->publish(gimbal_msg);
  //   }
  //   catch (const std::exception &ex)
  //   {
  //     RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 20,
  //                           "Error while receiving data: %s", ex.what());
  //     port_->closePort();
  //     rclcpp::sleep_for(std::chrono::seconds(1));
  //     port_->openPort();
  //   }
  // }

  void SerialDriverNode::classify(uint8_t *data)
{
    // 首先打印包头信息进行验证
    Header packet_header;
    std::memcpy(&packet_header, data, sizeof(Header));
    
    RCLCPP_INFO(get_logger(), "Packet Header:");
    RCLCPP_INFO(get_logger(), "  Protocol ID: 0x%02X", packet_header.protocolID);
    RCLCPP_INFO(get_logger(), "  Data Length: %d", packet_header.dataLen);
    
    // 打印完整数据包的十六进制值
    int full_length = sizeof(Header) + packet_header.dataLen + 2;
    std::string hex_dump = "Packet Hex: ";
    for (int i = 0; i < full_length; i++) {
        char byte_str[4];
        snprintf(byte_str, sizeof(byte_str), "%02X ", data[i]);
        hex_dump += byte_str;
    }
    RCLCPP_INFO(get_logger(), "%s", hex_dump.c_str());

    // 尝试解析云台消息
    InfantryGimbalMsg packet = bufferToStruct<InfantryGimbalMsg>(data);
    if (packet.header.protocolID != INFANTRY_GIMBAL_MSG) {
        RCLCPP_ERROR(get_logger(), 
                     "Invalid protocol ID: expected 0x%02X, got 0x%02X",
                     INFANTRY_GIMBAL_MSG, packet.header.protocolID);
        return;
    }
    
    try
    {
        RCLCPP_INFO(get_logger(), "classify: pitch:%f yaw:%f", packet.pitch, packet.yaw);
        classify_pkg_sum++;
        double pitch, yaw;
        pitch = packet.pitch;
        yaw = packet.yaw;
        auto gimbal_msg = interfaces::msg::Gimbal();
        gimbal_msg.gimbal_pitch = pitch;
        gimbal_msg.gimbal_yaw = yaw;
        
        gimbal_pub_->publish(gimbal_msg);
    }
    catch (const std::exception &ex)
    {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 20,
                            "Error while receiving data: %s", ex.what());
        // 不推荐直接重开串口，可能造成连接中断
        // port_->closePort();
        // rclcpp::sleep_for(std::chrono::seconds(1));
        // port_->openPort();
    }
}

PkgState SerialDriverNode::decode()
{
    // 获取当前接收缓冲区大小
    int size = receive_buffer.size();
    // RCLCPP_ERROR(get_logger(), "decode: size:%d", size);
    
    // 遍历接收缓冲区，查找有效数据包起始标志
    for (int i = 0; i < size; i++)
    {
        // 检查当前字节是否为包头起始标志 (0xFF)
        if (receive_buffer[i] == 0xFF)
        {
            // 检查是否有足够数据包含完整包头
            if (i + int(sizeof(Header)) > size)
            {
                // 包头数据不完整，返回状态
                // RCLCPP_ERROR(get_logger(), "包头数据不完整");
                return PkgState::HEADER_INCOMPLETE;
            }

            // 将包头数据复制到解码缓冲区
            std::copy(receive_buffer.begin() + i,
                     receive_buffer.begin() + i + sizeof(Header), 
                     decodeBuffer);
            // 不再进行CRC校验，直接视为有效包头
            // 保留包头中的CRC位置作为占位符
            // uint8_t header_crc_placeholder1 = decodeBuffer[sizeof(Header) - 2];
            // uint8_t header_crc_placeholder2 = decodeBuffer[sizeof(Header) - 1];

            // 将包头字节数据转换为结构体
            this->header = bufferToStruct<Header>(decodeBuffer);

            // 计算完整包长度（保持与原始协议相同的长度）
            int full_pkg_length = sizeof(Header) + header.dataLen + 2;
            
            // 检查是否有足够数据包含完整负载
            if (i + full_pkg_length > size)
            {
                // RCLCPP_ERROR(get_logger(), "i = %d  full_pkg_length = %d  size = %d", i, full_pkg_length, size);
                // RCLCPP_ERROR(get_logger(), "负载数据不完整");
                // 负载数据不完整，返回状态
                return PkgState::PAYLOAD_INCOMPLETE;
            }

            // 将完整数据包复制到解码缓冲区
            std::copy(receive_buffer.begin() + i,
                     receive_buffer.begin() + i + full_pkg_length,
                     decodeBuffer);

            // 不再进行CRC校验，直接获取占位符
            // uint8_t pkg_crc_placeholder1 = decodeBuffer[full_pkg_length - 2];
            // uint8_t pkg_crc_placeholder2 = decodeBuffer[full_pkg_length - 1];

            // 成功解码完整数据包
            try
            {
                // 从接收缓冲区移除已处理的数据包
                receive_buffer.erase(
                    receive_buffer.begin(),
                    receive_buffer.begin() + i + full_pkg_length);
            }
            catch (const std::exception &ex)
            {
                RCLCPP_ERROR(get_logger(), "Error creating serial port:  %s", ex.what());
            }

            pkg_sum++;  // 增加成功解码包计数

            // RCLCPP_INFO(get_logger(), "success");

            // 调试信息：计算并输出各种统计信息
            std::chrono::high_resolution_clock::time_point end =
                std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> time = end - start;

#if DEBUG_SERIAL_DRIVER
            RCLCPP_INFO(
                get_logger(),
                "Package processed - Rate: %f hz, Total packages: %d",
                double(pkg_sum) / time.count(),
                pkg_sum);
#endif

            // 调用分类函数处理解码后的数据
            classify(decodeBuffer);
            
            // 返回完成状态
            return PkgState::COMPLETE;
        }
        RCLCPP_ERROR(get_logger(), "No header found");
    }
    // 未找到任何有效包头，清空缓冲区
    receive_buffer.erase(receive_buffer.begin(), receive_buffer.end());

    // 返回其他状态
    return PkgState::OTHER;
}

//   PkgState SerialDriverNode::decode()
//   {
//     int size = receive_buffer.size();

//     for (int i = 0; i < size; i++)
//     {
//       if (receive_buffer[i] == 0xFF)
//       {
//         if (i + int(sizeof(Header)) > size)
//         {
//           return PkgState::HEADER_INCOMPLETE;
//         }

//         std::copy(receive_buffer.begin() + i,
//                   receive_buffer.begin() + i + sizeof(Header), decodeBuffer);
//         crc_ok_header =
//             crc16::Verify_CRC16_Check_Sum(decodeBuffer, sizeof(Header));

//         if (!crc_ok_header)
//         {
//           RCLCPP_ERROR(get_logger(), "CRC error in header !");
//           error_sum_header++;
//           try
//           {
//             receive_buffer.erase(receive_buffer.begin() + i,
//                                  receive_buffer.begin() + i + sizeof(Header));
//           }
//           catch (const std::exception &ex)
//           {
//             RCLCPP_ERROR(get_logger(), "Error creating serial port:  %s",
//                          ex.what());
//           }

//           return PkgState::CRC_HEADER_ERRROR;
//         }

//         this->header = bufferToStruct<Header>(decodeBuffer);

//         // pkg length = payload(dataLen) + header len (include header crc) +
//         // 2crc
//         if (i + int(header.dataLen) + int(sizeof(Header)) + 2 > size)
//         {
//           return PkgState::PAYLOAD_INCOMPLETE;
//         }

//         std::copy(
//             receive_buffer.begin() + i,
//             receive_buffer.begin() + i + header.dataLen + sizeof(Header) + 2,
//             decodeBuffer);
//         crc_ok = crc16::Verify_CRC16_Check_Sum(
//             decodeBuffer, header.dataLen + sizeof(Header) + 2);

//         if (!crc_ok)
//         {
//           error_sum_payload++;
//           // payload error
//           try
//           {
//             // check if there is a coming pkg
//             for (int j = i + 1;
//                  j < int(header.dataLen) + int(sizeof(Header)) + 2 + i; j++)
//             {
//               if (receive_buffer[j] == 0xFF)
//               {
//                 if (j + sizeof(Header) >
//                     header.dataLen + sizeof(Header) + 2 + i)
//                 {
//                   receive_buffer.erase(receive_buffer.begin(),
//                                        receive_buffer.begin() + j);
//                   return PkgState::HEADER_INCOMPLETE;
//                 }
//                 std::copy(receive_buffer.begin() + i,
//                           receive_buffer.begin() + i + sizeof(Header),
//                           decodeBuffer);
//                 crc_ok_header =
//                     crc16::Verify_CRC16_Check_Sum(decodeBuffer, sizeof(Header));

//                 if (!crc_ok_header)
//                 {
//                   receive_buffer.erase(
//                       receive_buffer.begin(),
//                       receive_buffer.begin() + j + sizeof(Header));
//                   j += sizeof(Header) - 1;
//                   continue;
//                 }

//                 this->header = bufferToStruct<Header>(decodeBuffer);

//                 if (j + sizeof(Header) + header.dataLen + 2)
//                 {
//                   receive_buffer.erase(receive_buffer.begin(),
//                                        receive_buffer.begin() + j);
//                   return PkgState::PAYLOAD_INCOMPLETE;
//                 }

//                 std::copy(receive_buffer.begin() + i,
//                           receive_buffer.begin() + i + header.dataLen +
//                               sizeof(Header) + 2,
//                           decodeBuffer);
//                 crc_ok = crc16::Verify_CRC16_Check_Sum(
//                     decodeBuffer, header.dataLen + sizeof(Header) + 2);

//                 if (!crc_ok)
//                 {
//                   RCLCPP_ERROR(get_logger(), "CRC error in payload !");
//                   receive_buffer.erase(receive_buffer.begin(),
//                                        receive_buffer.begin() + j +
//                                            sizeof(Header) + header.dataLen + 2);
//                   j += sizeof(Header) + header.dataLen + 2 - 1;
//                   continue;
//                 }
//               }
//             }
//             receive_buffer.erase(receive_buffer.begin(),
//                                  receive_buffer.begin() + i + header.dataLen +
//                                      sizeof(Header) + 2);
//           }
//           catch (const std::exception &ex)
//           {
//             RCLCPP_ERROR(get_logger(), "Error creating serial port:  %s",
//                          ex.what());
//           }

//           return PkgState::CRC_PKG_ERROR;
//         }

//         // complete
//         try
//         {
//           receive_buffer.erase(
//               receive_buffer.begin(),
//               receive_buffer.begin() + i + header.dataLen + sizeof(Header) + 2);
//         }
//         catch (const std::exception &ex)
//         {
//           RCLCPP_ERROR(get_logger(), "Error creating serial port:  %s",
//                        ex.what());
//         }

//         pkg_sum++;

//         std::chrono::high_resolution_clock::time_point end =
//             std::chrono::high_resolution_clock::now();
//         std::chrono::duration<double> time = end - start;

// #if DEBUG_SERIAL_DRIVER
//         RCLCPP_INFO(
//             get_logger(),
//             "crc error rate : %f pkg sum rate: %f hz classify_pkg_sum "
//             "rate:%f hz,read_sum rate: %f "
//             "transmit hz :%f  time : %f \n",
//             float(error_sum_header + error_sum_payload) / float(pkg_sum),
//             double(pkg_sum) / time.count(),
//             double(classify_pkg_sum) / time.count(),
//             float(read_sum) * 11 / time.count(), trans_pkg_sum / time.count(),
//             time.count());
// #endif

//         classify(decodeBuffer);
//         return PkgState::COMPLETE;
//       }
//     }
//     receive_buffer.erase(receive_buffer.begin(), receive_buffer.end());
//     return PkgState::OTHER;
//   }

  // communicate with RV
  void SerialDriverNode::GimbalCmdCallback(interfaces::msg::CenterDelta::SharedPtr msg)
  {
    try
    {
      uint8_t buffer[sizeof(GimbalCommand)];
      GimbalCommand packet;
      packet.header.dataLen = sizeof(GimbalCommand) - sizeof(Header) - 2;
      packet.header.protocolID = GIMBAL_CMD;
      packet.is_shoot = msg->is_shoot;
      packet.pitch_cmd = msg->pitch;
      packet.yaw_cmd = msg->yaw;
      crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet),
                                    sizeof(Header));
      crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet),
                                    sizeof(GimbalCommand));

      structToBuffer(packet, buffer);
      std::lock_guard<std::mutex> lockf(transmit_mutex);
      transmit_buffer.insert(transmit_buffer.end(), buffer,
                             buffer + sizeof(GimbalCommand));
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(get_logger(), "Error while sending data: %s", e.what());
      port_->closePort();
      rclcpp::sleep_for(std::chrono::seconds(1));
      port_->openPort();
    }
  }


  
  void SerialDriverNode::getParam()
  {

    int baud_rate{};
    bool flowcontrol = false;
    auto pt = Parity::NONE;
    auto sb = rm_serial_driver::StopBit::ONE;

    try
    {
      device_name_ =
          declare_parameter<std::string>("device_name", "/dev/ttyACM0");
    }
    catch (rclcpp::ParameterTypeException &ex)
    {
      RCLCPP_ERROR(get_logger(), "The device name provided was invalid");
      throw ex;
    }

    try
    {
      baud_rate = declare_parameter<int>("baud_rate", 961200);
    }
    catch (rclcpp::ParameterTypeException &ex)
    {
      RCLCPP_ERROR(get_logger(), "The baud_rate provided was invalid");
      throw ex;
    }

    try
    {
      flowcontrol = declare_parameter<bool>("flow_control", false);
    }
    catch (rclcpp::ParameterTypeException &ex)
    {
      RCLCPP_ERROR(get_logger(), "The flow_control provided was invalid");
      throw ex;
    }

    try
    {
      const auto pt_string = declare_parameter<std::string>("parity", "none");

      if (pt_string == "none")
      {
        pt = Parity::NONE;
      }
      else if (pt_string == "odd")
      {
        pt = Parity::ODD;
      }
      else if (pt_string == "even")
      {
        pt = Parity::EVEN;
      }
      else
      {
        throw std::invalid_argument{
            "The parity parameter must be one of: none, odd, or even."};
      }
    }
    catch (rclcpp::ParameterTypeException &ex)
    {
      RCLCPP_ERROR(get_logger(), "The parity provided was invalid");
      throw ex;
    }

    try
    {
      const auto sb_string = declare_parameter<std::string>("stop_bits", "1.0");

      if (sb_string == "1" || sb_string == "1.0")
      {
        sb = StopBit::ONE;
      }
      else if (sb_string == "1.5")
      {
        sb = StopBit::ONE_POINT_FIVE;
      }
      else if (sb_string == "2" || sb_string == "2.0")
      {
        sb = StopBit::TWO;
      }
      else
      {
        throw std::invalid_argument{
            "The stop_bits parameter must be one of: 1, 1.5, or 2."};
      }
    }
    catch (rclcpp::ParameterTypeException &ex)
    {
      RCLCPP_ERROR(get_logger(), "The stop_bits provided was invalid");
      throw ex;
    }

    config_ = std::make_unique<SerialConfig>(baud_rate, 8, flowcontrol, sb, pt,
                                             device_name_);
  }
}
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rm_serial_driver::SerialDriverNode)