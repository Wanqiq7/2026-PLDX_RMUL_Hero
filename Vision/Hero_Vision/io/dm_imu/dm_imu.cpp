#include "dm_imu.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <string>
#include <thread>
#include <vector>

#include "tools/crc.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace io
{
namespace
{
std::string GetEnvOrDefault(const char * key, const char * default_value)
{
  const char * v = std::getenv(key);
  if (v == nullptr || v[0] == '\0') return default_value;
  return std::string(v);
}

int GetEnvIntOrDefault(const char * key, int default_value)
{
  const char * v = std::getenv(key);
  if (v == nullptr || v[0] == '\0') return default_value;
  try {
    return std::stoi(v);
  } catch (...) {
    return default_value;
  }
}

void SendUsbShortcut(serial::Serial & serial, const std::array<uint8_t, 4> & cmd)
{
  serial.write(const_cast<uint8_t *>(cmd.data()), cmd.size());
}
}  // namespace

DM_IMU::DM_IMU() : queue_(5000)
{
  init_serial();
  rec_thread_ = std::thread(&DM_IMU::get_imu_data_thread, this);

  // 等待两帧，确保 data_ahead_ / data_behind_ 有效；避免“连错设备/波特率不匹配”时卡死很久
  const int init_timeout_ms = GetEnvIntOrDefault("SP_DM_IMU_INIT_TIMEOUT_MS", 10000);
  const auto init_deadline =
    std::chrono::steady_clock::now() + std::chrono::milliseconds(init_timeout_ms);

  auto wait_one_frame = [&](IMUData & out, const char * which) {
    auto last_hint = std::chrono::steady_clock::now();
    while (queue_.empty()) {
      if (std::chrono::steady_clock::now() > init_deadline) {
        tools::logger()->error(
          "[DM_IMU] 等待 {} 超时（{}ms）。大概率是端口号/波特率不匹配或设备未输出。"
          " 可用环境变量覆盖：SP_DM_IMU_PORT、SP_DM_IMU_BAUD。",
          which, init_timeout_ms);
        std::exit(1);
      }

      // 每秒提示一次，避免用户误以为程序“卡死”
      if (std::chrono::steady_clock::now() - last_hint > std::chrono::seconds(1)) {
        tools::logger()->warn(
          "[DM_IMU] 仍在等待 {}...（提示：若看到大量 ASCII 日志，通常是连错设备；若全乱码，通常是波特率不对）",
          which);
        last_hint = std::chrono::steady_clock::now();
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    queue_.pop(out);
  };

  wait_one_frame(data_ahead_, "第1帧");
  wait_one_frame(data_behind_, "第2帧");
  tools::logger()->info("[DM_IMU] initialized");
}

DM_IMU::~DM_IMU()
{
  stop_thread_ = true;
  if (rec_thread_.joinable()) {
    rec_thread_.join();
  }
  if (serial_.isOpen()) {
    serial_.close();
  }
}

void DM_IMU::init_serial()
{
  try {
    const std::string port = GetEnvOrDefault("SP_DM_IMU_PORT", "/dev/ttyACM0");
    const int baud = GetEnvIntOrDefault("SP_DM_IMU_BAUD", 921600);

    tools::logger()->info("[DM_IMU] opening serial port={} baud={}", port, baud);

    serial_.setPort(port);
    serial_.setBaudrate(baud);
    serial_.setFlowcontrol(serial::flowcontrol_none);
    serial_.setParity(serial::parity_none);  //default is parity_none
    serial_.setStopbits(serial::stopbits_one);
    serial_.setBytesize(serial::eightbits);
    serial::Timeout time_out = serial::Timeout::simpleTimeout(20);
    serial_.setTimeout(time_out);
    serial_.open();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    tools::logger()->info("[DM_IMU] serial port opened (Passive Mode)");

    // 根据说明书：USB 通道默认主动输出四类数据，但可能被用户/上位机关闭。
    // 这里在启动时主动发送“开启输出”的快捷指令，避免只收到 0x01(加速度) 导致初始化长期阻塞。
    const bool send_config = GetEnvIntOrDefault("SP_DM_IMU_SEND_CONFIG", 1) != 0;
    if (send_config) {
      try {
        SendUsbShortcut(serial_, {0xAA, 0x01, 0x14, 0x0D});  // 开启加速度输出
        SendUsbShortcut(serial_, {0xAA, 0x01, 0x15, 0x0D});  // 开启角速度输出
        SendUsbShortcut(serial_, {0xAA, 0x01, 0x16, 0x0D});  // 开启欧拉角输出
        SendUsbShortcut(serial_, {0xAA, 0x01, 0x17, 0x0D});  // 开启四元数输出
      } catch (const std::exception & e) {
        tools::logger()->warn("[DM_IMU] failed to send USB config cmds: {}", e.what());
      } catch (...) {
        tools::logger()->warn("[DM_IMU] failed to send USB config cmds: unknown error");
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

  catch (serial::IOException & e) {
    tools::logger()->error("[DM_IMU] failed to open serial port: {}", e.what());
    std::exit(1);
  }
}

void DM_IMU::get_imu_data_thread()
{
  std::vector<uint8_t> buffer;
  buffer.reserve(2048);

  const bool push_from_euler = GetEnvIntOrDefault("SP_DM_IMU_PUSH_FROM_EULER", 1) != 0;
  const std::string euler_unit = GetEnvOrDefault("SP_DM_IMU_EULER_UNIT", "deg");
  const bool euler_in_deg = (euler_unit != "rad");
  constexpr double kDeg2Rad = 0.017453292519943295;  // pi / 180
  const bool accept_without_crc = GetEnvIntOrDefault("SP_DM_IMU_ACCEPT_WITHOUT_CRC", 1) != 0;

  while (!stop_thread_) {
    if (!serial_.isOpen()) {
      tools::logger()->warn("In get_imu_data_thread,imu serial port unopen");
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }

    auto available = serial_.available();
    if (available > 0) {
      std::vector<uint8_t> tmp(available);
      auto got = serial_.read(tmp.data(), available);
      buffer.insert(buffer.end(), tmp.begin(), tmp.begin() + got);
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // 逐帧解析：支持 0x01/0x02/0x03(19B) 与 0x04(23B)
    std::size_t i = 0;
    while (i < buffer.size()) {
      if (buffer.size() - i < 4) break;  // 不足以判头

      if (buffer[i] != 0x55 || buffer[i + 1] != 0xAA) {
        i += 1;
        continue;
      }

      uint8_t pkg_type = buffer[i + 3];
      std::size_t frame_len = 0;
      std::size_t data_len = 0;
      if (pkg_type >= 0x01 && pkg_type <= 0x03) {
        frame_len = 19;
        data_len = 12;
      } else if (pkg_type == 0x04) {
        frame_len = 23;
        data_len = 16;
      } else {
        i += 1;
        continue;
      }

      if (buffer.size() - i < frame_len) break;  // 数据不够

      const bool end_ok = (buffer[i + frame_len - 1] == 0x0A);

      bool crc_ok = true;
      if (!accept_without_crc) {
        uint16_t calc_len = 4 + data_len;
        // 兼容两种常见 CRC16（CCITT/IBM），以及 CRC 字节序 LE/BE
        uint16_t crc_ccitt = tools::Get_CRC16(buffer.data() + i, calc_len);
        uint16_t crc_ibm = tools::get_crc16(buffer.data() + i, calc_len);
        uint16_t crc_recv_le =
          static_cast<uint16_t>(buffer[i + frame_len - 3]) |
          (static_cast<uint16_t>(buffer[i + frame_len - 2]) << 8);
        uint16_t crc_recv_be =
          (static_cast<uint16_t>(buffer[i + frame_len - 3]) << 8) |
          static_cast<uint16_t>(buffer[i + frame_len - 2]);
        auto matches = [&](uint16_t v) { return v == crc_recv_le || v == crc_recv_be; };
        crc_ok = matches(crc_ccitt) || matches(crc_ibm);
      }

      if (end_ok && (crc_ok || accept_without_crc)) {
        if (pkg_type == 0x01) {
          std::memcpy(&data.accx, &buffer[i + 4], 4);
          std::memcpy(&data.accy, &buffer[i + 8], 4);
          std::memcpy(&data.accz, &buffer[i + 12], 4);
        } else if (pkg_type == 0x02) {
          std::memcpy(&data.gyrox, &buffer[i + 4], 4);
          std::memcpy(&data.gyroy, &buffer[i + 8], 4);
          std::memcpy(&data.gyroz, &buffer[i + 12], 4);
        } else if (pkg_type == 0x03) {
          std::memcpy(&data.roll, &buffer[i + 4], 4);
          std::memcpy(&data.pitch, &buffer[i + 8], 4);
          std::memcpy(&data.yaw, &buffer[i + 12], 4);

          // 许多 DM-IMU 配置默认只输出欧拉角（roll/pitch/yaw）而不输出四元数。
          // 为了避免初始化长时间阻塞，这里将欧拉角转换为四元数并推入队列。
          if (push_from_euler) {
            double roll = static_cast<double>(data.roll);
            double pitch = static_cast<double>(data.pitch);
            double yaw = static_cast<double>(data.yaw);
            if (std::isfinite(roll) && std::isfinite(pitch) && std::isfinite(yaw)) {
              if (euler_in_deg) {
                roll *= kDeg2Rad;
                pitch *= kDeg2Rad;
                yaw *= kDeg2Rad;
              }
              Eigen::Matrix3d R = tools::rotation_matrix({yaw, pitch, roll});
              Eigen::Quaterniond q(R);
              q.normalize();
              auto timestamp = std::chrono::steady_clock::now();
              queue_.push({q, timestamp});
            }
          }
        } else if (pkg_type == 0x04) {
          float w, x, y, z;
          std::memcpy(&w, &buffer[i + 4], 4);
          std::memcpy(&x, &buffer[i + 8], 4);
          std::memcpy(&y, &buffer[i + 12], 4);
          std::memcpy(&z, &buffer[i + 16], 4);
          if (std::isfinite(w) && std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
            Eigen::Quaterniond q(w, x, y, z);
            q.normalize();
            auto timestamp = std::chrono::steady_clock::now();
            queue_.push({q, timestamp});
          }
        }
        i += frame_len;
      } else {
        i += 1;
      }
    }
    // 统一丢弃已处理的缓冲前缀
    buffer.erase(buffer.begin(), buffer.begin() + i);

    // 防止极端情况下缓冲过大
    if (buffer.size() > 4096) buffer.clear();
  }
}

Eigen::Quaterniond DM_IMU::imu_at(std::chrono::steady_clock::time_point timestamp)
{
  if (data_behind_.timestamp < timestamp) data_ahead_ = data_behind_;

  while (true) {
    queue_.pop(data_behind_);
    if (data_behind_.timestamp > timestamp) break;
    data_ahead_ = data_behind_;
  }

  Eigen::Quaterniond q_a = data_ahead_.q.normalized();
  Eigen::Quaterniond q_b = data_behind_.q.normalized();
  auto t_a = data_ahead_.timestamp;
  auto t_b = data_behind_.timestamp;
  auto t_c = timestamp;
  std::chrono::duration<double> t_ab = t_b - t_a;
  std::chrono::duration<double> t_ac = t_c - t_a;

  // 四元数插值
  auto k = t_ac / t_ab;
  Eigen::Quaterniond q_c = q_a.slerp(k, q_b).normalized();

  return q_c;
}

}  // namespace io
