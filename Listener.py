import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class SerialListener(Node):
    def __init__(self):
        super().__init__('serial_listener')

        # Publishers pour les différents types de messages
        self.pub_l = self.create_publisher(String, 'chatter', 10)
        self.pub_switch_gauche = self.create_publisher(String, 'switch_gauche', 10)
        self.pub_switch_droite = self.create_publisher(String, 'switch_droite', 10)

        # Connexion série
        try:
            self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
            self.get_logger().info("Port série ouvert avec succès.")
        except serial.SerialException as e:
            self.get_logger().error(f"Erreur d’ouverture du port série : {e}")
            return

        # Timer de lecture série
        self.timer = self.create_timer(0.001, self.read_serial)

    def read_serial(self):
        try:
            if self.ser.in_waiting > 0:
                message = self.ser.readline().decode('utf-8', errors='ignore').strip()

                if ":" in message:
                    parts = message.split(":", 1)
                    label = parts[0].strip()
                    data = parts[1].strip()

                    if label == "L":
                        msg = String()
                        msg.data = data
                        self.pub_l.publish(msg)
                        self.get_logger().info(f"L = {data}")

                    elif label == "SWITCHG":
                        msg = String()
                        msg.data = data
                        self.pub_switch_gauche.publish(msg)
                        self.get_logger().info(f"Levier gauche : {data}")

                    elif label == "SWITCHD":
                        msg = String()
                        msg.data = data
                        self.pub_switch_droite.publish(msg)
                        self.get_logger().info(f"Levier droite : {data}")

        except Exception as e:
            self.get_logger().error(f"Erreur lors de la lecture série : {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SerialListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
