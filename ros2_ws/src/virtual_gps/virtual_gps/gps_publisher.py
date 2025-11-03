import rclpy 
from rclpy.node import Node
from std_msgs.msg import String 
import random
import serial
import time

class GPSPublisher(Node):
    def __init__(self, serial_port="/tmp/ttyV0", baudrate=9600):
        super().__init__("gps_publisher")
        self.publisher_ = self.create_publisher(String, "gps_data", 10)

        self.ser = serial.Serial(serial_port, baudrate)
        time.sleep(2)

        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        latitude = round(random.uniform(-90, 90), 6)
        longitude = round(random.uniform(-180, 180), 6)
        altitude = round(random.uniform(0, 5000), 2)

        msg = String()
        msg.data = f"Lat: {latitude}, Lon: {longitude}, Alt: {altitude}m"

        self.publisher_.publish(msg)

        if self.ser.is_open:
            self.ser.write((msg.data + "\n").encode("utf-8"))

        self.get_logger().info(f"KONUŞUYORUM: {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    gps_publisher = GPSPublisher()
    rclpy.spin(gps_publisher)
    gps_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
