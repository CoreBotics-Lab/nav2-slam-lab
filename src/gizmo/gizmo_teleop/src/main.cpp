#include <QApplication>
#include <rclcpp/rclcpp.hpp>
#include <csignal>
#include <memory>

#include "gizmo_teleop/joy_node.hpp"
#include "gizmo_teleop/main_window.hpp"

static std::shared_ptr<QApplication> app_ptr;

void sigint_handler(int sig)
{
  (void)sig;
  if (app_ptr) {
    app_ptr->quit();
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  app_ptr = std::make_shared<QApplication>(argc, argv);

  auto node = std::make_shared<JoyNode>();

  MainWindow window(node);
  window.show();

  std::signal(SIGINT, sigint_handler);

  int exit_code = app_ptr->exec();

  rclcpp::shutdown();
  return exit_code;
}
