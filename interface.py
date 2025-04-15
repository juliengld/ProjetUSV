import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import tkinter as tk
from tkinter import ttk
import threading

class ROSInterface(Node):
    def __init__(self):
        super().__init__('gui_control_node')

        self.pub = self.create_publisher(String, 'user_command', 10)

        # Souscriptions
        self.create_subscription(String, 'switch_gauche', self.cb_switch_gauche, 10)
        self.create_subscription(String, 'switch_droite', self.cb_switch_droite, 10)
        self.create_subscription(String, 'chatter', self.cb_chatter, 10)

        # États reçus
        self.switch_gauche = "Inconnu"
        self.switch_droite = "Inconnu"
        self.current_speed = "0.0"
        self.direction = "?"

        # Callback GUI pour mise à jour
        self.gui_callback = None

    def send_speed_command(self, speed):
        msg = String()
        msg.data = f"SPEED: {speed:.2f}"
        self.pub.publish(msg)
        self.get_logger().info(f"[GUI] Commande envoyée : {msg.data}")

    def cb_switch_gauche(self, msg):
        self.switch_gauche = msg.data
        if self.gui_callback:
            self.gui_callback()

    def cb_switch_droite(self, msg):
        self.switch_droite = msg.data
        if self.gui_callback:
            self.gui_callback()

    def cb_chatter(self, msg):
        try:
            value = float(msg.data)
            self.current_speed = f"{value:.2f}"
            self.direction = "tire" if value > 0 else "rembobine" if value < 0 else "immobile"
        except ValueError:
            pass
        if self.gui_callback:
            self.gui_callback()

class GUI:
    def __init__(self, ros_node):
        self.ros = ros_node
        self.ros.gui_callback = self.update_status

        self.root = tk.Tk()
        self.root.title("Contrôle Robot")

        self.speed_entry = tk.Entry(self.root, width=10)
        self.speed_entry.insert(0, "0.3")
        self.send_btn = tk.Button(self.root, text="Envoyer", command=self.send_speed)

        self.label_gauche = tk.Label(self.root, text="Levier gauche : Inconnu")
        self.label_droite = tk.Label(self.root, text="Levier droite : Inconnu")
        self.label_speed = tk.Label(self.root, text="Vitesse actuelle : 0.0 m/s")
        self.label_direction = tk.Label(self.root, text="Direction : ?")

        self.speed_entry.grid(row=0, column=0, padx=10, pady=10)
        self.send_btn.grid(row=0, column=1, padx=10)

        self.label_gauche.grid(row=1, column=0, columnspan=2, sticky="w", padx=10)
        self.label_droite.grid(row=2, column=0, columnspan=2, sticky="w", padx=10)
        self.label_speed.grid(row=3, column=0, columnspan=2, sticky="w", padx=10)
        self.label_direction.grid(row=4, column=0, columnspan=2, sticky="w", padx=10)

    def send_speed(self):
        try:
            speed = float(self.speed_entry.get())
            self.ros.send_speed_command(speed)
        except ValueError:
            print("Entrée invalide")

    def update_status(self):
        self.label_gauche.config(text=f"Levier gauche : {self.ros.switch_gauche}")
        self.label_droite.config(text=f"Levier droite : {self.ros.switch_droite}")
        self.label_speed.config(text=f"Vitesse actuelle : {self.ros.current_speed} m/s")
        self.label_direction.config(text=f"Direction : {self.ros.direction}")

    def run(self):
        self.root.mainloop()

def main():
    rclpy.init()
    ros_node = ROSInterface()

    gui = GUI(ros_node)
    gui_thread = threading.Thread(target=gui.run, daemon=True)
    gui_thread.start()

    try:
        rclpy.spin(ros_node)
    except KeyboardInterrupt:
        pass
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
