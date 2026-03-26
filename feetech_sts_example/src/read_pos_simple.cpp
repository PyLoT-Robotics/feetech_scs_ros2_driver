// Simple position reader for STS motors (ID 1 and ID 2)

#include <feetech_sts_interface/feetech_sts_interface.hpp>
#include <unistd.h>
#include <iomanip>

using namespace feetech_sts_interface;

int main(int argc, char ** argv)
{
  std::string port_name = "/dev/ttyACM3";
  int baudrate = 1000000;

  if (argc >= 2) {
    port_name = argv[1];
  }
  if (argc >= 3) {
    baudrate = std::stoi(argv[2]);
  }

  auto port_handler = std::make_shared<h6x_serial_interface::PortHandler>(port_name);
  auto packet_handler = std::make_shared<feetech_sts_interface::PacketHandler>(port_handler);

  port_handler->configure(baudrate);
  if (!port_handler->open()) {
    std::cerr << "Failed to open " << port_name << std::endl;
    return EXIT_FAILURE;
  }
  std::cout << "Port opened: " << port_name << std::endl;

  using namespace std::chrono_literals;
  while (true) {
    for (int id = 1; id <= 2; id++) {
      int16_t val = 0;
      if (packet_handler->readPos(id, val)) {
        float angle = STS3032::data2angle(val);
        std::cout << "ID " << id << ": "
                  << std::fixed << std::setprecision(2) << angle << " deg"
                  << "  (raw: " << val << ")";
      } else {
        std::cout << "ID " << id << ": read failed";
      }
      if (id == 1) {
        std::cout << "  |  ";
      }
    }
    std::cout << std::endl;
    std::this_thread::sleep_for(200ms);
  }

  port_handler->close();
  return EXIT_SUCCESS;
}
