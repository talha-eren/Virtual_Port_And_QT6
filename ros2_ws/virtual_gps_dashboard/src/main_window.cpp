#include "main_window.hpp"
#include <QDebug>
#include <QTimer>
#include <cstdlib>

MainWindow::MainWindow(std::shared_ptr<rclcpp::Node> node, QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow), node_(node)
{
    ui->setupUi(this);

    connect(ui->btn_connect, &QPushButton::clicked, this, &MainWindow::onConnectClicked);
    connect(ui->btn_disconnect, &QPushButton::clicked, this, &MainWindow::onDisconnectClicked);

    // ROS2 spin timer
    QTimer *rosTimer = new QTimer(this);
    connect(rosTimer, &QTimer::timeout, [this]() {
        if (node_) rclcpp::spin_some(node_);
    });
    rosTimer->start(50); // 20 Hz
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::onConnectClicked()
{
    if (!gps_subscription_) {
        ui->textEdit->append("Connected to ROS2 node...");

        gps_subscription_ = node_->create_subscription<std_msgs::msg::String>(
            "gps_data",
            10,
            std::bind(&MainWindow::gpsCallback, this, std::placeholders::_1)
        );
    }
}

void MainWindow::onDisconnectClicked()
{
    if (gps_subscription_) {
        ui->textEdit->append("Disconnected from ROS2 node...");
        gps_subscription_.reset();
    }
}

void MainWindow::gpsCallback(const std_msgs::msg::String::SharedPtr msg)
{
    QString data = QString::fromStdString(msg->data);
    ui->textEdit->append(data);

    double altitude = rand() % 1000;
    ui->value_alt->setText(QString::number(altitude));

    QStringList parts = data.split(",");
    if (parts.size() >= 2) {
        ui->value_lat->setText(parts[0].split(":")[1].trimmed());
        ui->value_lon->setText(parts[1].split(":")[1].trimmed());
    }
}
