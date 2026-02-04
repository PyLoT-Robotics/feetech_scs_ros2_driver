// Copyright 2026
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <thread>

#include <feetech_sts_interface/feetech_sts_interface.hpp>
#include <h6x_serial_interface/port_handler.hpp>

#define DEFAULT_PORT "/dev/ttyACM0"
#define DEFAULT_BAUDRATE 1000000

struct ServoRange {
  double min_angle = std::numeric_limits<double>::max();
  double max_angle = std::numeric_limits<double>::lowest();
  int speed = 1000;
  int acceleration = 50;
};

int main(int argc, char ** argv)
{
  std::string port_name = DEFAULT_PORT;
  int baudrate = DEFAULT_BAUDRATE;
  std::string output_file = "servo_config.yaml";

  if (argc >= 2) {
    port_name = argv[1];
  }
  if (argc >= 3) {
    baudrate = std::stoi(argv[2]);
  }
  if (argc >= 4) {
    output_file = argv[3];
  }

  std::cout << "Creating servo configuration file: " << output_file << std::endl;
  std::cout << "Port: " << port_name << ", Baudrate: " << baudrate << std::endl;

  auto port_handler = std::make_shared<h6x_serial_interface::PortHandler>(port_name);
  auto packet_handler = std::make_shared<feetech_sts_interface::PacketHandler>(port_handler);

  port_handler->configure(baudrate);
  if (!port_handler->open()) {
    std::cerr << "Failed to open port" << std::endl;
    return EXIT_FAILURE;
  }

  std::this_thread::sleep_for(std::chrono::seconds(1));

  // Scan for servos
  std::cout << "Scanning for servos (ID 1-6)..." << std::endl;
  std::vector<u_char> found_servos;

  for (u_char id = 1; id <= 6; ++id) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    int ping_result = packet_handler->ping(id);
    if (ping_result > 0) {
      std::cout << "Found servo with ID: " << static_cast<int>(id) << std::endl;
      found_servos.push_back(id);
    }
  }

  if (found_servos.empty()) {
    std::cerr << "No servos found!" << std::endl;
    return EXIT_FAILURE;
  }

  std::cout << "\n=== Servo Range Calibration ===\n";
  std::cout << "You will manually move each servo through its full range of motion.\n";
  std::cout << "The program will record the min and max angles.\n\n";

  // Store recorded ranges for each servo
  std::map<u_char, ServoRange> servo_ranges;

  // Calibrate each servo one by one
  for (const auto& id : found_servos) {
    std::cout << "\n--- Servo ID " << static_cast<int>(id) << " ---\n";
    
    // Turn OFF torque so the servo can be moved freely
    packet_handler->setTorque(id, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    std::cout << "Torque disabled. You can now move the servo freely.\n";
    std::cout << "Move servo ID " << static_cast<int>(id) 
              << " through its FULL range of motion.\n";
    std::cout << "Press Enter when ready to start recording (10 seconds)...";
    std::cin.get();
    
    ServoRange range;
    auto start_time = std::chrono::steady_clock::now();
    auto duration = std::chrono::seconds(10);
    int sample_count = 0;
    
    std::cout << "Recording... (move the servo now!)\n";
    
    while (std::chrono::steady_clock::now() - start_time < duration) {
      int16_t pos_data = 0;
      if (packet_handler->readPos(id, pos_data)) {
        double angle_deg = feetech_sts_interface::STS3032::data2angle(pos_data);
        
        if (angle_deg < range.min_angle) {
          range.min_angle = angle_deg;
        }
        if (angle_deg > range.max_angle) {
          range.max_angle = angle_deg;
        }
        
        sample_count++;
        
        // Display current angle and range every 20 samples
        if (sample_count % 20 == 0) {
          auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::steady_clock::now() - start_time).count();
          std::cout << "Time: " << elapsed << "s | Current: " << angle_deg 
                    << "° | Range: [" << range.min_angle << "° ~ " 
                    << range.max_angle << "°]\n";
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    
    std::cout << "\nRecording complete!\n";
    std::cout << "Recorded range: [" << range.min_angle << "° ~ " 
              << range.max_angle << "°]\n";
    std::cout << "Samples collected: " << sample_count << "\n";
    
    servo_ranges[id] = range;
  }

  std::cout << "\n=== All servos calibrated! ===\n";
  std::cout << "Saving configuration to: " << output_file << "\n\n";

  // Create YAML configuration
  std::ofstream yaml_file(output_file);
  if (!yaml_file.is_open()) {
    std::cerr << "Failed to open output file: " << output_file << std::endl;
    return EXIT_FAILURE;
  }

  yaml_file << "# Feetech STS Servo Configuration\n";
  yaml_file << "# Auto-generated configuration file\n";
  yaml_file << "# Calibrated by manual movement\n\n";
  yaml_file << "servos:\n";

  for (const auto& id : found_servos) {
    const auto& range = servo_ranges[id];
    double center = (range.min_angle + range.max_angle) / 2.0;
    double half_range = (range.max_angle - range.min_angle) / 2.0;
    
    yaml_file << "  - id: " << static_cast<int>(id) << "\n";
    yaml_file << "    name: \"servo_" << static_cast<int>(id) << "\"\n";
    yaml_file << "    # Physical angle limits (calibrated)\n";
    yaml_file << "    min_angle: " << range.min_angle << "\n";
    yaml_file << "    max_angle: " << range.max_angle << "\n";
    yaml_file << "    # Input angle mapping (logical angles)\n";
    yaml_file << "    # Adjust these based on robot's coordinate system\n";
    yaml_file << "    input_min: " << -half_range << "  # e.g., for yaw: -90.0\n";
    yaml_file << "    input_max: " << half_range << "  # e.g., for yaw: 90.0\n";
    yaml_file << "    # Default speed and acceleration\n";
    yaml_file << "    default_speed: " << range.speed << "\n";
    yaml_file << "    default_acceleration: " << range.acceleration << "\n";
    yaml_file << "\n";
  }

  yaml_file << "# Communication settings\n";
  yaml_file << "port: \"" << port_name << "\"\n";
  yaml_file << "baudrate: " << baudrate << "\n";
  yaml_file << "\n";
  yaml_file << "# Publishing rate for current state (Hz)\n";
  yaml_file << "publish_rate: 10.0\n";

  yaml_file.close();

  std::cout << "\nConfiguration saved to: " << output_file << std::endl;
  std::cout << "Calibrated " << found_servos.size() << " servo(s)\n";
  std::cout << "\nCalibration results:\n";
  for (const auto& id : found_servos) {
    const auto& range = servo_ranges[id];
    std::cout << "  Servo " << static_cast<int>(id) << ": [" 
              << range.min_angle << "° ~ " << range.max_angle << "°]\n";
  }

  // Port will be closed automatically by shared_ptr destructor
  
  return EXIT_SUCCESS;
}
