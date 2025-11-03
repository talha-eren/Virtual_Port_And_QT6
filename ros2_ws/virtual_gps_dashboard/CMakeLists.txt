cmake_minimum_required(VERSION 3.8)
project(virtual_gps_dashboard)

# ROS2 bağımlılıkları
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Qt6 bağımlılıkları
find_package(Qt6 REQUIRED COMPONENTS Widgets)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

include_directories(
  include
)

# Uygulamayı oluştur
add_executable(${PROJECT_NAME}
  src/main.cpp
  src/main_window.cpp
  src/main_window.ui
)

# Qt kütüphanelerini plain biçimde bağla (PRIVATE / PUBLIC YOK)
target_link_libraries(${PROJECT_NAME}
  Qt6::Widgets
)

# ROS bağımlılıklarını ekle
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
)

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
