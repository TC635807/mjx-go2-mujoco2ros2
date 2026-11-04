#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>

static std::atomic<int> g_n{0};

static void handler(const void* msg) {
  (void)msg;
  g_n.fetch_add(1);
}

int main(int argc, char** argv) {
  int domain = (argc > 1) ? std::atoi(argv[1]) : 1;
  std::string iface = (argc > 2) ? argv[2] : "lo";
  int seconds = (argc > 3) ? std::atoi(argv[3]) : 8;
  unitree::robot::ChannelFactory::Instance()->Init(domain, iface);
  unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::LowCmd_> sub("rt/lowcmd", handler, 10);
  sub.InitChannel();
  for (int i = 0; i < seconds * 10; i++) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  std::printf("[sdk2-cxx] received %d LowCmd on rt/lowcmd (domain=%d iface=%s)\n",
              g_n.load(), domain, iface.c_str());
  return g_n.load() > 0 ? 0 : 1;
}
