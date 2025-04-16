import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class CommandPublisher(Node):
    def __init__(self):
        super().__init__('command_publisher')

        # Publisher vers 'chatter' (facultatif)
        self.publisher_ = self.create_publisher(String, 'chatter', 10)

        # Subscriber pour recevoir les consignes de l’utilisateur
        self.subscription = self.create_subscription(
            String,
            'user_command',
            self.command_callback,
            10
        )

        self.last_command = None  # Pour éviter les doublons

        try:
            self.serial_conn = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
            self.get_logger().info("Connexion série établie avec l'Arduino.")
        except serial.SerialException:
            self.get_logger().error("Impossible de se connecter à l'Arduino.")
            self.serial_conn = None

        # Envoi d’une première commande par défaut
        self.default_speed = 0.3
        self.timer = self.create_timer(0.5, self.send_initial_command)
        self.sent_initial = False

    def send_initial_command(self):
        if not self.sent_initial:
            default_msg = String()
            default_msg.data = f"SPEED: {self.default_speed}"
            self.send_command(default_msg)
            self.sent_initial = True

    def command_callback(self, msg: String):
        new_command = msg.data.strip()
        if new_command != self.last_command:
            self.send_command(msg)
        else:
            self.get_logger().info(f"Commande déjà envoyée : {new_command}")

    def send_command(self, msg: String):
        self.last_command = msg.data.strip()

        # Envoi série
        if self.serial_conn:
            self.serial_conn.write((self.last_command + "\n").encode())

        # Publication ROS (si besoin)
        self.publisher_.publish(msg)

        self.get_logger().info(f"Nouvelle commande envoyée : {self.last_command}")

def main(args=None):
    rclpy.init(args=args)
    node = CommandPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
