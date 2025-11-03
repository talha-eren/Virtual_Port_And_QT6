#ifndef MAIN_WINDOW_HPP
#define MAIN_WINDOW_HPP

#include <QMainWindow>
#include <QTimer>
#include <QString>
#include "ui_main_window.h"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(std::shared_ptr<rclcpp::Node> node, QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void gpsCallback(const std_msgs::msg::String::SharedPtr msg);
    void onConnectClicked();
    void onDisconnectClicked();

private:
    Ui::MainWindow *ui;
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gps_subscription_;
};

#endif // MAIN_WINDOW_HPP
