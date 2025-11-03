import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class GPSSubscriber(Node):
    def __init__(self, serial_port="/tmp/ttyV1", baudrate=9600):
        super().__init__("gps_subscriber")
        self.subscription = self.create_subscription(
            String,
            "gps_data",
            self.listener_callback,
            10
        )

        self.ser = serial.Serial(serial_port, baudrate)

    def listener_callback(self, msg):
        self.get_logger().info(f"DİNLİYORUM: {msg.data}")  

        if self.ser.is_open:
            self.ser.write((msg.data + "\n").encode("utf-8"))

def main(args=None):
    rclpy.init(args=args)
    gps_subscriber = GPSSubscriber()
    rclpy.spin(gps_subscriber)
    gps_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
