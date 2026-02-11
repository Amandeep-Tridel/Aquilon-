#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"

#include "MOOS/libMOOS/Comms/MOOSCommClient.h"
#include "MOOS/libMOOS/Comms/MOOSMsg.h"

struct MoosToRosMapping {
  std::string moos_var;
  std::string ros_topic;
  std::string msg_type;  // "double" or "string"
};

struct RosToMoosMapping {
  std::string ros_topic;
  std::string moos_var;
  std::string msg_type;
};

class MoosRosBridge : public rclcpp::Node
{
public:
  MoosRosBridge()
  : Node("moos_ros2_bridge")
  {
    // Declare parameters
    this->declare_parameter<std::string>("moos_host", "localhost");
    this->declare_parameter<int>("moos_port", 9000);
    this->declare_parameter<std::string>("moos_name", "moos_ros2_bridge");
    this->declare_parameter<double>("poll_rate_hz", 10.0);
    this->declare_parameter<std::vector<std::string>>(
      "moos_to_ros", std::vector<std::string>{});
    this->declare_parameter<std::vector<std::string>>(
      "ros_to_moos", std::vector<std::string>{});

    // Read parameters
    auto moos_host = this->get_parameter("moos_host").as_string();
    auto moos_port = this->get_parameter("moos_port").as_int();
    auto moos_name = this->get_parameter("moos_name").as_string();
    auto poll_rate_hz = this->get_parameter("poll_rate_hz").as_double();
    auto m2r_strs = this->get_parameter("moos_to_ros").as_string_array();
    auto r2m_strs = this->get_parameter("ros_to_moos").as_string_array();

    // Parse MOOS->ROS mappings
    for (const auto & entry : m2r_strs) {
      auto m = parse_mapping(entry);
      if (!m.has_value()) {
        RCLCPP_ERROR(this->get_logger(),
          "Bad moos_to_ros mapping: '%s' (expected MOOS_VAR:/topic:type)", entry.c_str());
        continue;
      }
      auto & [f1, f2, f3] = m.value();
      moos_to_ros_.push_back({f1, f2, f3});

      if (f3 == "double") {
        dbl_pubs_[f1] = this->create_publisher<std_msgs::msg::Float64>(f2, 10);
      } else {
        str_pubs_[f1] = this->create_publisher<std_msgs::msg::String>(f2, 10);
      }
      RCLCPP_INFO(this->get_logger(), "MOOS->ROS: %s -> %s [%s]",
        f1.c_str(), f2.c_str(), f3.c_str());
    }

    // Parse ROS->MOOS mappings
    for (const auto & entry : r2m_strs) {
      auto m = parse_mapping(entry);
      if (!m.has_value()) {
        RCLCPP_ERROR(this->get_logger(),
          "Bad ros_to_moos mapping: '%s' (expected /topic:MOOS_VAR:type)", entry.c_str());
        continue;
      }
      auto & [ros_topic, moos_var, msg_type] = m.value();
      ros_to_moos_.push_back({ros_topic, moos_var, msg_type});

      if (msg_type == "double") {
        std::string mv = moos_var;
        dbl_subs_.push_back(
          this->create_subscription<std_msgs::msg::Float64>(
            ros_topic, 10,
            [this, mv](const std_msgs::msg::Float64::SharedPtr msg) {
              std::lock_guard<std::mutex> lock(mtx_);
              moos_.Notify(mv, msg->data);
            }));
      } else {
        std::string mv = moos_var;
        str_subs_.push_back(
          this->create_subscription<std_msgs::msg::String>(
            ros_topic, 10,
            [this, mv](const std_msgs::msg::String::SharedPtr msg) {
              std::lock_guard<std::mutex> lock(mtx_);
              moos_.Notify(mv, msg->data);
            }));
      }
      RCLCPP_INFO(this->get_logger(), "ROS->MOOS: %s -> %s [%s]",
        ros_topic.c_str(), moos_var.c_str(), msg_type.c_str());
    }

    // MOOS connection callback — registers variables on (re)connect
    moos_.SetOnConnectCallBack(on_connect, this);

    // Start MOOS client (non-blocking — spawns internal comms thread)
    moos_.Run(moos_host, static_cast<int>(moos_port), moos_name);
    RCLCPP_INFO(this->get_logger(), "MOOS client connecting to %s:%ld as '%s'",
      moos_host.c_str(), moos_port, moos_name.c_str());

    // Timer to poll MOOS mail
    auto period_ms = std::chrono::milliseconds(
      static_cast<int>(1000.0 / poll_rate_hz));
    timer_ = this->create_wall_timer(period_ms,
      std::bind(&MoosRosBridge::poll_moos, this));
  }

  ~MoosRosBridge() override { moos_.Close(true); }

private:
  static bool on_connect(void * param)
  {
    auto * self = static_cast<MoosRosBridge *>(param);
    RCLCPP_INFO(self->get_logger(), "Connected to MOOSDB");
    std::lock_guard<std::mutex> lock(self->mtx_);
    for (const auto & m : self->moos_to_ros_) {
      self->moos_.Register(m.moos_var, 0.0);
      RCLCPP_INFO(self->get_logger(), "  Registered: %s", m.moos_var.c_str());
    }
    return true;
  }

  void poll_moos()
  {
    MOOSMSG_LIST mail;
    {
      std::lock_guard<std::mutex> lock(mtx_);
      if (!moos_.IsConnected()) return;
      moos_.Fetch(mail);
    }

    for (auto & msg : mail) {
      const auto & key = msg.GetKey();
      if (msg.IsDouble()) {
        auto it = dbl_pubs_.find(key);
        if (it != dbl_pubs_.end()) {
          std_msgs::msg::Float64 ros_msg;
          ros_msg.data = msg.GetDouble();
          it->second->publish(ros_msg);
        }
      } else if (msg.IsString()) {
        auto it = str_pubs_.find(key);
        if (it != str_pubs_.end()) {
          std_msgs::msg::String ros_msg;
          ros_msg.data = msg.GetString();
          it->second->publish(ros_msg);
        }
      }
    }
  }

  std::optional<std::tuple<std::string, std::string, std::string>>
  parse_mapping(const std::string & entry)
  {
    auto p1 = entry.find(':');
    if (p1 == std::string::npos) return std::nullopt;
    auto p2 = entry.find(':', p1 + 1);
    if (p2 == std::string::npos) return std::nullopt;
    auto f1 = entry.substr(0, p1);
    auto f2 = entry.substr(p1 + 1, p2 - p1 - 1);
    auto f3 = entry.substr(p2 + 1);
    if (f1.empty() || f2.empty() || (f3 != "double" && f3 != "string"))
      return std::nullopt;
    return std::make_tuple(f1, f2, f3);
  }

  CMOOSCommClient moos_;
  std::mutex mtx_;

  std::vector<MoosToRosMapping> moos_to_ros_;
  std::vector<RosToMoosMapping> ros_to_moos_;

  std::map<std::string, rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> dbl_pubs_;
  std::map<std::string, rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> str_pubs_;
  std::vector<rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr> dbl_subs_;
  std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> str_subs_;

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MoosRosBridge>());
  rclcpp::shutdown();
  return 0;
}
