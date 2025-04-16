import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class CommandPublisher(Node):
    def __init__(self):
        super().__init__('command_publisher')  
        self.publisher_ = self.create_publisher(String, 'chatter', 10)  
        self.timer = self.create_timer(2.0, self.publish_command)
        self.state = False

        self.speed_value = -0.1

        # Initialisation du port série
        try:
            self.serial_conn = serial.Serial('/dev/ttyACM0', 115200, timeout=1)  
        except serial.SerialException:
            self.get_logger().error("Impossible de se connecter à l'arduino")  
            self.serial_conn = None

    def publish_command(self):
        msg_speed = String()
        msg_speed.data = f"SPEED: {self.speed_value}"
        self.publisher_.publish(msg_speed)

        if self.serial_conn:
            self.serial_conn.write((msg_speed.data + "\n").encode())

        self.get_logger().info(f'Commande envoyée: {msg_speed.data}')
        self.state = not self.state

def main(args=None):
    rclpy.init(args=args)
    node = CommandPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':  
    main()

