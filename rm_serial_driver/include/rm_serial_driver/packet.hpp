#include <sys/cdefs.h>

#include <algorithm>
#include <boost/cstdint.hpp>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <map>
#include <variant>
#include <vector>


namespace rm_serial_driver
{
// Ax stands for msg received (msg from mcu to nuc)
// Bx stands for msg sent (msg from nuc to mcu)

  enum CommunicationType : uint8_t
  {
    INFANTRY_GIMBAL_MSG = 0xA1,
    CHASSIS_MSG = 0xA2,
    SENTRY_GIMBAL_MSG = 0xA3,
    FIELD_MSG = 0xA4,

    GIMBAL_CMD = 0xB0,
    CHASSIS_CMD = 0xB2,
    ACTION_CMD = 0xB3,
    SENTRY_LEFT_GIMBAL_CMD = 0xB4,
    SENTRY_RIGHT_GIMBAL_CMD = 0xB6,

    INFANTRY_AUTOAIM_DATA=0xB5,
  };

  // **************************** //
  // * protocol for ring buffer * //
  // **************************** //
  struct Header
  {
    uint8_t sof = 0xFFu;
    uint8_t dataLen = 0;
    uint8_t protocolID = 0;
    uint8_t crc_1;
    uint8_t crc_2;

    Header() = default;

  } __attribute__((packed));

  struct ChassisMsg
  {
    Header header;

    float xVel;
    float yVel;
    float wVel;

    uint8_t crc_3;
    uint8_t crc_4;

  } __attribute__((packed));

  struct FieldMsg
  {
    Header header;

    uint8_t game_state;
    uint16_t remain_time;
    uint16_t bullet_remain;
    uint8_t team; // 0 - red 1 - blue

    uint16_t red_hp[8];
    uint16_t blue_hp[8];
    bool in_combat;
    bool bullet_supply_available;
    bool shooter_locked;

    uint8_t crc_3;
    uint8_t crc_4;

  } __attribute__((packed));

  struct InfantryGimbalMsg // TWOCRC_GIMBAL_MSG, also 0xA2
  {
    Header header;
    float pitch;
    float yaw;
    uint8_t crc_3;
    uint8_t crc_4;

  } __attribute__((packed));

  struct SentryGimbalMsg // TWOCRC_GIMBALSTATUS_MSG, also 0xA3
  {
    Header header;

    uint8_t cur_cv_mode;
    uint8_t target_color;
    float bullet_speed;

    // small gimbal q
    float small_left_pitch;
    float small_left_yaw;
    float small_left_roll;

    float small_right_pitch;
    float small_right_yaw;
    float small_right_roll;

    // main gimbal q
    float main_yaw;


    uint8_t crc_3;
    uint8_t crc_4;

  } __attribute__((packed));

  struct GimbalCommand // TWOCRC_GIMBAL_CMD, also 0xB1
  {
    Header header;

    uint8_t is_shoot;
    float pitch_cmd;
    float yaw_cmd;

    uint8_t crc_3;
    uint8_t crc_4;

  } __attribute__((packed));

  struct ChassisCommand // TWOCRC_CHASSIS_CMD, also 0xB2
  {
    Header header;

    float vel_x;
    float vel_y;
    float vel_w;

//填充字节数，保证Command类型的数据包长度相等
    float reserved_1=0;
    float reserved_2=0;
    float reserved_3=0;
    uint8_t reserved_4=0;
    uint8_t reserved_5=0;

    uint8_t crc1;
    uint8_t crc2;

  } __attribute__((packed));

  struct ActionCommand // TWOCRC_ACTION_CMD, also 0xB3
  {
    Header header;

    bool scan;
    float spin;
    // bool cv_enable;

    //填充字节数，保证Command类型的数据包长度相等
    float reserved_1=0;
    float reserved_2=0;
    float reserved_3=0;
    float reserved_4=0;
    float reserved_5=0;
    uint8_t reserved_6=0;
    
    

    uint8_t crc_3;
    uint8_t crc_4;

  } __attribute__((packed));

  struct SentryLeftGimbalCommand
  {
    Header header;

    float l_pitch;
    float l_yaw;
    uint8_t l_is_shoot;//左

    // float r_pitch;
    // float r_yaw;
    // uint8_t r_is_shoot;//右

    // float main_pitch;
    // float main_yaw;

    uint8_t crc_3;
    uint8_t crc_4;

  } __attribute__((packed));

   struct SentryRightGimbalCommand
  {
    Header header;

    float r_pitch;
    float r_yaw;
    uint8_t r_is_shoot;//左

    // float r_pitch;
    // float r_yaw;
    // uint8_t r_is_shoot;//右

    // float main_pitch;
    // float main_yaw;

    uint8_t crc_3;
    uint8_t crc_4;

  } __attribute__((packed));




  struct InfantryAutoData
  {
    /* data */
    Header header;
    bool tracking : 1;
    uint8_t id : 3;         // 0-outpost 6-guard 7-base
    uint8_t armors_num : 3; // 2-balance 3-outpost 4-normal
    uint8_t reserved : 1;
    float x;
    float y;
    float z;
    float yaw;
    float vx;
    float vy;
    float vz;
    float v_yaw;
    float r1;
    float r2;
    float dz;
    uint8_t crc_3;
    uint8_t crc_4;

  } __attribute__((packed));

  using ProtocoType = std::variant<
      InfantryGimbalMsg, ChassisMsg, SentryGimbalMsg, FieldMsg, GimbalCommand, ChassisCommand, ActionCommand,
      SentryLeftGimbalCommand,SentryRightGimbalCommand,InfantryAutoData>;
  std::map<CommunicationType, ProtocoType> protocolMap;

  template <typename T>
  inline T bufferToStruct(const uint8_t *buffer)
  {
    T result;
    std::memcpy(&result, buffer, sizeof(T));
    return result;
  }

  template <typename T>
  void structToBuffer(const T &inputStruct, uint8_t *outputArray)
  {
    std::memcpy(outputArray, reinterpret_cast<const uint8_t *>(&inputStruct), sizeof(T));
  }

} // namespace rm_serial_driver
