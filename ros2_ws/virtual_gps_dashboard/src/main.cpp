#include "main_window.hpp"
#include <QApplication>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("virtual_gps_dashboard");

    QApplication app(argc, argv);
    MainWindow w(node);
    w.show();

    int ret = app.exec();
    rclcpp::shutdown();
    return ret;
}
