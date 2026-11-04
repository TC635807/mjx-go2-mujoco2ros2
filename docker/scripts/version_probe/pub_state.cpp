#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>

int main(int argc, char** argv) {
  int domain = (argc > 1) ? std::atoi(argv[1]) : 1;
  std::string iface = (argc > 2) ? argv[2] : "lo";
  unitree::robot::ChannelFactory::Instance()->Init(domain, iface);
  unitree::robot::ChannelPublisher<unitree_go::msg::dds_::LowState_> pub("rt/lowstate");
  pub.InitChannel();

  unitree_go::msg::dds_::LowState_ m{};
  m.head()[0] = 0xFE;
  m.head()[1] = 0xEF;
  m.motor_state()[0].q() = 0.1234f;
  m.motor_state()[1].q() = 0.5678f;
  m.imu_state().rpy()[1] = 0.01f;

  const int n = 3000;   // 6 s @ 500 Hz
  for (int i = 0; i < n; i++) {
    m.tick() = static_cast<uint32_t>(i);
    pub.Write(m);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  std::printf("[sdk2-cxx] published %d LowState on rt/lowstate (domain=%d iface=%s)\n", n, domain, iface.c_str());
  return 0;
}
