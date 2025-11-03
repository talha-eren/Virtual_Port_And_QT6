# Virtual Port and Qt6 - ROS2 GPS Dashboard

A ROS2-based virtual GPS system with a Qt6 graphical user interface. This project simulates GPS data transmission through virtual serial ports and displays it in a modern Qt dashboard.

## 📹 Demo Video

You can watch the output demonstration video below. The video file (`input.webm`) is located in the `video/` directory.

<video 
https://github.com/user-attachments/assets/5c6e631d-444f-4c0d-a75a-4a3f25c947b7
</video>

**Note:** If the video above doesn't play, you can download and view it directly from the [`video/input.webm`](video/input.webm) file.

## 📋 Overview

This project consists of three main components:

1. **GPS Publisher** - Publishes simulated GPS data (latitude, longitude, altitude) to a ROS2 topic
2. **GPS Subscriber** - Subscribes to GPS data and forwards it to a virtual serial port
3. **Qt6 Dashboard** - A graphical interface that displays GPS data in real-time

## 🏗️ Project Structure

```
.
├── ros2_ws/
│   ├── src/
│   │   └── virtual_gps/           # Python ROS2 package for GPS publishing/subscribing
│   │       ├── virtual_gps/
│   │       │   ├── gps_publisher.py
│   │       │   └── gps_subscriber.py
│   │       └── package.xml
│   └── virtual_gps_dashboard/     # Qt6 C++ ROS2 package for GUI
│       ├── src/
│       │   ├── main.cpp
│       │   ├── main_window.cpp
│       │   ├── main_window.hpp
│       │   └── main_window.ui
│       ├── CMakeLists.txt
│       └── package.xml
└── video/
    └── input.webm                 # Demo video output
```

## 🚀 Prerequisites

- ROS2 (tested with ROS2 Humble/Jazzy)
- Python 3.10+
- Qt6 development libraries
- CMake 3.8+
- pyserial library

### Installation

```bash
# Install ROS2 (if not already installed)
# Follow official ROS2 installation guide

# Install Qt6 dependencies
sudo apt-get install qt6-base-dev

# Install Python dependencies
pip3 install pyserial
```

## 🔧 Building the Project

1. Navigate to the ROS2 workspace:
```bash
cd ros2_ws
```

2. Build the workspace:
```bash
colcon build
```

3. Source the workspace:
```bash
source install/setup.bash
```

## 🎯 Usage

### 1. Setup Virtual Serial Ports

First, create virtual serial ports using `socat`:

```bash
# Terminal 1 - Create virtual serial ports
socat -d -d pty,raw,echo=0 pty,raw,echo=0
```

This will output two PTY paths (e.g., `/dev/pts/2` and `/dev/pts/3`). Note these paths or create symlinks:

```bash
sudo ln -sf /dev/pts/2 /tmp/ttyV0  # For publisher
sudo ln -sf /dev/pts/3 /tmp/ttyV1  # For subscriber
```

### 2. Run the GPS Publisher

```bash
# Terminal 2
ros2 run virtual_gps gps_publisher
```

This will start publishing GPS data to the `/gps_data` topic and to the virtual serial port `/tmp/ttyV0`.

### 3. Run the GPS Subscriber (Optional)

```bash
# Terminal 3
ros2 run virtual_gps gps_subscriber
```

This will subscribe to the `/gps_data` topic and forward messages to the virtual serial port `/tmp/ttyV1`.

### 4. Launch the Qt6 Dashboard

```bash
# Terminal 4
ros2 run virtual_gps_dashboard virtual_gps_dashboard
```

The dashboard window will open. Click "Connect" to start receiving GPS data and see it displayed in real-time.

## 📊 Features

- **Real-time GPS Data Visualization**: View latitude, longitude, and altitude in the dashboard
- **Virtual Serial Communication**: Simulates GPS data transmission through virtual serial ports
- **ROS2 Integration**: Uses ROS2 topics for inter-process communication
- **Modern Qt6 GUI**: User-friendly interface with connect/disconnect functionality

## 🎥 Video Output

The project output demonstration can be viewed in the [`video/input.webm`](video/input.webm) file. This video shows the Qt6 dashboard in action, displaying real-time GPS coordinates and altitude values.

## 📝 License

This project is licensed under the MIT License - see the package.xml files for details.

## 👤 Author

**Talha Eren**
- Email: bilikcitalha@gmail.com
- GitHub: [@talha-eren](https://github.com/talha-eren)

## 🤝 Contributing

Contributions, issues, and feature requests are welcome! Feel free to check the [issues page](https://github.com/talha-eren/Virtual_Port_And_QT6/issues).

## 📚 Resources

- [ROS2 Documentation](https://docs.ros.org/)
- [Qt6 Documentation](https://doc.qt.io/qt-6/)
- [ROS2 Qt Integration Guide](https://ros-planning.github.io/moveit_tutorials/doc/move_group_python_interface/move_group_python_interface_tutorial.html)

