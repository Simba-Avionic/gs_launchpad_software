#include <signal.h>

#include "rclcpp/rclcpp.hpp"

#include "Oberon.hpp"

void signalHandler(int signum)
{
  rclcpp::shutdown();
  
  if (signum == SIGINT)
    exit(0);

  exit(signum);
}

int main(int argc, char ** argv)
{
  signal(SIGINT, signalHandler);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Oberon>());
  rclcpp::shutdown();
  return 0;
}
