#include <chrono>
#include <cmath>
#include <csignal>
#include <fstream>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <feetech_sts_interface/feetech_sts_interface.hpp>
#include <h6x_serial_interface/port_handler.hpp>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using sensor_msgs::msg::JointState;

#define DEFAULT_PORT "/dev/ttyACM0"
#define DEFAULT_BAUDRATE 1000000
#define BROADCAST_ID 0

#define STS_TORQUE_ENABLE      40
#define STS_GOAL_POSITION_L   42
#define STS_GOAL_SPEED_L      46

struct ServoLimits {
  double min_angle = 0.0;
  double max_angle = 300.0;
  int default_speed = 1000;
  int default_acceleration = 50;
  // Input angle mapping (logical angles)
  double input_min = 0.0;
  double input_max = 300.0;
};

// Simple YAML parser for servo config
std::map<u_char, ServoLimits> loadServoConfig(const std::string& filename) {
  std::map<u_char, ServoLimits> config;
  std::ifstream file(filename);
  
  if (!file.is_open()) {
    std::cerr << "Warning: Could not open config file: " << filename << std::endl;
    return config;
  }
  
  std::string line;
  u_char current_id = 0;
  ServoLimits current_limits;
  bool in_servo_section = false;
  
  while (std::getline(file, line)) {
    // Remove leading/trailing whitespace
    size_t start = line.find_first_not_of(" \t");
    if (start == std::string::npos) continue;
    line = line.substr(start);
    
    // Skip comments
    if (line[0] == '#') continue;
    
    // Check for servo ID
    if (line.find("- id:") != std::string::npos) {
      if (in_servo_section && current_id > 0) {
        config[current_id] = current_limits;
      }
      size_t pos = line.find(':');
      if (pos != std::string::npos) {
        current_id = std::stoi(line.substr(pos + 1));
        current_limits = ServoLimits(); // Reset to defaults
        in_servo_section = true;
      }
    }
    else if (in_servo_section) {
      if (line.find("min_angle:") != std::string::npos) {
        size_t pos = line.find(':');
        current_limits.min_angle = std::stod(line.substr(pos + 1));
      }
      else if (line.find("max_angle:") != std::string::npos) {
        size_t pos = line.find(':');
        current_limits.max_angle = std::stod(line.substr(pos + 1));
      }
      else if (line.find("default_speed:") != std::string::npos) {
        size_t pos = line.find(':');
        current_limits.default_speed = std::stoi(line.substr(pos + 1));
      }
      else if (line.find("default_acceleration:") != std::string::npos) {
        size_t pos = line.find(':');
        current_limits.default_acceleration = std::stoi(line.substr(pos + 1));
      }
      else if (line.find("input_min:") != std::string::npos) {
        size_t pos = line.find(':');
        current_limits.input_min = std::stod(line.substr(pos + 1));
      }
      else if (line.find("input_max:") != std::string::npos) {
        size_t pos = line.find(':');
        current_limits.input_max = std::stod(line.substr(pos + 1));
      }
    }
  }
  
  // Add last servo
  if (in_servo_section && current_id > 0) {
    config[current_id] = current_limits;
  }
  
  file.close();
  return config;
}

// Convert input angle (logical) to servo angle (physical)
double mapInputToServo(double input_angle, const ServoLimits& limits) {
  // Linear mapping from input range to servo range
  double input_range = limits.input_max - limits.input_min;
  double servo_range = limits.max_angle - limits.min_angle;
  
  if (input_range == 0) {
    return limits.min_angle;
  }
  
  // Normalize input to 0-1 range
  double normalized = (input_angle - limits.input_min) / input_range;
  
  // Clamp to 0-1
  if (normalized < 0.0) normalized = 0.0;
  if (normalized > 1.0) normalized = 1.0;
  
  // Map to servo range
  return limits.min_angle + normalized * servo_range;
}

// Convert servo angle (physical) back to input angle (logical)
double mapServoToInput(double servo_angle, const ServoLimits& limits) {
  // Reverse mapping from servo range to input range
  double input_range = limits.input_max - limits.input_min;
  double servo_range = limits.max_angle - limits.min_angle;
  
  if (servo_range == 0) {
    return (limits.input_min + limits.input_max) / 2.0;
  }
  
  // Normalize servo angle to 0-1 range
  double normalized = (servo_angle - limits.min_angle) / servo_range;
  
  // Clamp to 0-1
  if (normalized < 0.0) normalized = 0.0;
  if (normalized > 1.0) normalized = 1.0;
  
  // Map to input range
  return limits.input_min + normalized * input_range;
}

// Global pointers for signal handler
std::shared_ptr<feetech_sts_interface::PacketHandler> g_packet_handler;
std::shared_ptr<std::set<u_char>> g_torque_enabled_ids;

// Signal handler for graceful shutdown
void signalHandler(int signum) {
  RCLCPP_INFO(rclcpp::get_logger("feetech_joint_state_node"), 
              "Signal %d received. Disabling torque...", signum);
  
  if (g_packet_handler && g_torque_enabled_ids) {
    for (const auto& id : *g_torque_enabled_ids) {
      try {
        g_packet_handler->setTorque(id, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      } catch (const std::exception& e) {
        RCLCPP_WARN(rclcpp::get_logger("feetech_joint_state_node"),
                    "Exception disabling torque for servo %d: %s", id, e.what());
      }
    }
    RCLCPP_INFO(rclcpp::get_logger("feetech_joint_state_node"),
                "All torques disabled");
  }
  
  rclcpp::shutdown();
  exit(signum);
}



class FeetechJointStateNode : public rclcpp::Node
{
public:
  FeetechJointStateNode(
    std::shared_ptr<feetech_sts_interface::PacketHandler> packet_handler,
    std::shared_ptr<std::set<u_char>> torque_enabled_ids)
  : Node("feetech_joint_state_node"),
    packet_handler_(packet_handler),
    torque_enabled_ids_(torque_enabled_ids)
  {
    RCLCPP_INFO(this->get_logger(), "Feetech JointState control node started");

    this->declare_parameter("qos_depth", 10);
    this->declare_parameter("publish_rate", 10.0); // Hz
    this->declare_parameter("config_file", "servo_config.yaml");
    
    int qos_depth = this->get_parameter("qos_depth").as_int();
    double publish_rate = this->get_parameter("publish_rate").as_double();
    std::string config_file = this->get_parameter("config_file").as_string();
    
    // Load servo configuration
    servo_limits_ = loadServoConfig(config_file);
    if (!servo_limits_.empty()) {
      RCLCPP_INFO(this->get_logger(), "Loaded configuration for %zu servo(s)", servo_limits_.size());
      for (const auto& [id, limits] : servo_limits_) {
        RCLCPP_INFO(this->get_logger(), 
                    "Servo %d: [%.1f° ~ %.1f°]", 
                    id, limits.min_angle, limits.max_angle);
      }
    } else {
      RCLCPP_WARN(this->get_logger(), "No servo limits loaded, using defaults");
    }

    auto qos = rclcpp::QoS(rclcpp::KeepLast(qos_depth))
                 .reliable()
                 .durability_volatile();

    joint_state_sub_ =
      this->create_subscription<JointState>(
        "joint_command",
        qos,
        std::bind(&FeetechJointStateNode::jointStateCallback, this, std::placeholders::_1)
      );
    
    joint_state_pub_ =
      this->create_publisher<JointState>("joint_states_raw", qos);
    
    // Timer to publish current positions
    auto timer_period = std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate));
    timer_ = this->create_wall_timer(
      timer_period,
      std::bind(&FeetechJointStateNode::publishCurrentState, this)
    );
  }

private:
  void publishCurrentState()
  {
    auto msg = JointState();
    msg.header.stamp = this->now();
    
    for (const auto& id : *torque_enabled_ids_) {
      int16_t pos_data = 0;
      if (packet_handler_->readPos(id, pos_data)) {
        // Convert data to angle in degrees (servo angle)
        double servo_angle_deg = feetech_sts_interface::STS3032::data2angle(pos_data);
        
        // Convert servo angle back to logical input angle
        double input_angle_deg = servo_angle_deg;
        if (servo_limits_.count(id) > 0) {
          const ServoLimits& limits = servo_limits_.at(id);
          input_angle_deg = mapServoToInput(servo_angle_deg, limits);
        }
        
        // Convert to radians
        double angle_rad = input_angle_deg * M_PI / 180.0;
        
        msg.name.push_back(std::to_string(id));
        msg.position.push_back(angle_rad);
      }
    }
    
    if (!msg.name.empty()) {
      joint_state_pub_->publish(msg);
    }
  }

  void jointStateCallback(const JointState::SharedPtr msg)
{
  if (msg->name.size() != msg->position.size()) {
    RCLCPP_WARN(this->get_logger(), "name and position size mismatch");
    return;
  }

  // Collect IDs, positions, speeds, and accelerations
  std::vector<u_char> ids;
  std::vector<int16_t> positions;
  std::vector<int16_t> speeds;
  std::vector<u_short> accelerations;

  for (size_t i = 0; i < msg->name.size(); ++i) {
    try {
      u_char id = static_cast<u_char>(std::stoi(msg->name[i]));

      // Enable torque for this servo if not already done
      if (torque_enabled_ids_->find(id) == torque_enabled_ids_->end()) {
        RCLCPP_INFO(this->get_logger(), "Enabling torque for servo ID: %d", id);
        packet_handler_->setTorque(id, 1);
        torque_enabled_ids_->insert(id);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }

      // rad → deg
      double input_angle_deg = msg->position[i] * 180.0 / M_PI;
      
      // Apply limits and mapping from config
      ServoLimits limits;
      double servo_angle_deg = input_angle_deg;
      
      if (servo_limits_.count(id) > 0) {
        limits = servo_limits_.at(id);
        
        // Map input angle to servo angle
        servo_angle_deg = mapInputToServo(input_angle_deg, limits);
        
        RCLCPP_DEBUG(this->get_logger(), 
                    "Servo %d: input %.1f° -> servo %.1f° (range: [%.1f, %.1f])", 
                    id, input_angle_deg, servo_angle_deg, 
                    limits.min_angle, limits.max_angle);
      }

      int16_t target_data =
        static_cast<int16_t>(feetech_sts_interface::STS3032::angle2data(servo_angle_deg));

      int16_t speed = limits.default_speed;
      u_short acceleration = limits.default_acceleration;

      ids.push_back(id);
      positions.push_back(target_data);
      speeds.push_back(speed);
      accelerations.push_back(acceleration);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Error parsing joint state: %s", e.what());
      return;
    }
  }

  if (ids.empty()) {
    RCLCPP_WARN(this->get_logger(), "No valid servo IDs found");
    return;
  }

  // Use syncWritePosEx for batch command
  bool ok = packet_handler_->syncWritePosEx(
    ids.data(),
    static_cast<u_char>(ids.size()),
    positions.data(),
    speeds.data(),
    accelerations.data()
  );

  if (!ok) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Failed to set positions for %zu servos",
      ids.size()
    );
  }
}


  rclcpp::Subscription<JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<JointState>::SharedPtr joint_state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<feetech_sts_interface::PacketHandler> packet_handler_;
  std::shared_ptr<std::set<u_char>> torque_enabled_ids_;
  std::map<u_char, ServoLimits> servo_limits_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto port_handler =
    std::make_shared<h6x_serial_interface::PortHandler>(DEFAULT_PORT);
  auto packet_handler =
    std::make_shared<feetech_sts_interface::PacketHandler>(port_handler);

  port_handler->configure(DEFAULT_BAUDRATE);

  if (!port_handler->open()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("feetech_joint_state_node"),
      "Failed to open port"
    );
    return -1;
  }

  std::this_thread::sleep_for(std::chrono::seconds(1));

  // Scan for connected servos and enable torque on all of them
  RCLCPP_INFO(
    rclcpp::get_logger("feetech_joint_state_node"),
    "Scanning for connected servos (ID 1-6)..."
  );
  
  auto torque_enabled_ids = std::make_shared<std::set<u_char>>();
  
  // Set global pointers for signal handler
  g_packet_handler = packet_handler;
  g_torque_enabled_ids = torque_enabled_ids;
  
  // Register signal handler for Ctrl+C
  std::signal(SIGINT, signalHandler);
  std::signal(SIGTERM, signalHandler);
  
  const u_char max_id = 6; // Scan only IDs 1-6 to avoid long delays
  
  for (u_char id = 1; id <= max_id; ++id) {
    // Wait before ping to ensure clean communication
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    // ping() returns true/1 on success, -1 on failure
    // Check explicitly for positive result
    int ping_result = packet_handler->ping(id);
    
    if (ping_result > 0) {
      RCLCPP_INFO(
        rclcpp::get_logger("feetech_joint_state_node"),
        "Found servo with ID: %d (ping result: %d)",
        id,
        ping_result
      );
      
      // Wait after successful ping
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      
      // Enable torque
      bool torque_result = packet_handler->setTorque(id, 1);
      
      // Wait after torque command
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      
      if (torque_result) {
        RCLCPP_INFO(
          rclcpp::get_logger("feetech_joint_state_node"),
          "Torque enabled for servo ID: %d",
          id
        );
        torque_enabled_ids->insert(id);
      } else {
        RCLCPP_WARN(
          rclcpp::get_logger("feetech_joint_state_node"),
          "Failed to enable torque for servo ID: %d",
          id
        );
      }
    }
  }
  
  RCLCPP_INFO(
    rclcpp::get_logger("feetech_joint_state_node"),
    "Servo initialization complete"
  );

  // Wait for torque to settle
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  RCLCPP_INFO(
    rclcpp::get_logger("feetech_joint_state_node"),
    "Total servos with enabled torque: %zu",
    torque_enabled_ids->size()
  );
  
  auto node =
    std::make_shared<FeetechJointStateNode>(packet_handler, torque_enabled_ids);

  rclcpp::spin(node);

  // 終了処理
  try {
    packet_handler->setTorque(BROADCAST_ID, 0);
  } catch (const std::exception& e) {
    RCLCPP_WARN(
      rclcpp::get_logger("feetech_joint_state_node"),
      "Exception during torque disable: %s",
      e.what()
    );
  }
  
  if (port_handler->open()) {
    port_handler->close();
  }

  rclcpp::shutdown();
  return 0;
}
