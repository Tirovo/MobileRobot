#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Float32
from sensor_msgs.msg import Image
import sys
import random
import numpy as np

from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QLabel, 
                            QLineEdit, QPushButton, QVBoxLayout, QHBoxLayout,
                            QGroupBox, QRadioButton, QComboBox)
from PyQt5.QtGui import QImage, QPixmap

class MainWindow(QMainWindow):
    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)
        
        # Initialisation de la fenêtre principale
        self.setMinimumSize(800, 600)
        self.setWindowTitle("Interface de Commande ROS2 pour Robot Mobile")
        
        # Variables ROS2
        self.i = 0
        self.latest_img_msg = None
        
        # Initialisation ROS2
        rclpy.init(args=None)
        self.node = Node('py_ihm_node')
        
        # Publishers
        self.publisher_cmd = self.node.create_publisher(String, 'pyqt_topic_send', 10)
        self.publisher_mode = self.node.create_publisher(Int32, 'robot_mode', 10)
        self.publisher_speed = self.node.create_publisher(Float32, 'robot_speed', 10)
        
        # Subscribers
        self.subscription_cmd = self.node.create_subscription(String, 'pyqt_topic_rec', self.listener_callback, 10)
        self.subscription_wheel_speed = self.node.create_subscription(Float32, 'wheel_speed', self.wheel_speed_callback, 10)
        self.subscription_robot_speed = self.node.create_subscription(Float32, 'robot_speed_feedback', self.robot_speed_callback, 10)
        self.subscription_obstacle = self.node.create_subscription(String, 'obstacle_detection', self.obstacle_callback, 10)
        self.subscription_camera = self.node.create_subscription(Image, 'camera_image', self.camera_callback, 10)
        
        # UI
        self.initUI()
        
        # ROS Timer
        self.timer = QTimer()
        self.timer.timeout.connect(self.onTimerTick)
        self.timer.start(5)

        # Random movement timer
        self.random_timer = QTimer()
        self.random_timer.timeout.connect(self.random_move)
        self.random_move_interval = 3000
        
    def initUI(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)

        # --- Contrôle ---
        control_panel = QGroupBox("Contrôle du Robot")
        control_layout = QVBoxLayout()

        mode_group = QGroupBox("Mode de Fonctionnement")
        mode_layout = QVBoxLayout()

        self.manual_radio = QRadioButton("Mode Manuel")
        self.random_radio = QRadioButton("Mode Aléatoire")
        self.tracking_radio = QRadioButton("Mode Tracking")

        self.manual_radio.setChecked(True)
        self.manual_radio.toggled.connect(self.on_mode_changed)
        self.random_radio.toggled.connect(self.on_mode_changed)
        self.tracking_radio.toggled.connect(self.on_mode_changed)

        mode_layout.addWidget(self.manual_radio)
        mode_layout.addWidget(self.random_radio)
        mode_layout.addWidget(self.tracking_radio)
        mode_group.setLayout(mode_layout)

        self.direction_group = QGroupBox("Direction (Mode Manuel)")
        direction_layout = QVBoxLayout()

        self.btn_forward = QPushButton("Avancer")
        self.btn_backward = QPushButton("Reculer")
        self.btn_left = QPushButton("Gauche")
        self.btn_right = QPushButton("Droite")
        self.btn_stop = QPushButton("Stop")

        self.btn_forward.clicked.connect(lambda: self.send_direction_cmd("forward"))
        self.btn_backward.clicked.connect(lambda: self.send_direction_cmd("backward"))
        self.btn_left.clicked.connect(lambda: self.send_direction_cmd("left"))
        self.btn_right.clicked.connect(lambda: self.send_direction_cmd("right"))
        self.btn_stop.clicked.connect(lambda: self.send_direction_cmd("stop"))

        direction_layout.addWidget(self.btn_forward)
        direction_layout.addWidget(self.btn_backward)
        direction_layout.addWidget(self.btn_left)
        direction_layout.addWidget(self.btn_right)
        direction_layout.addWidget(self.btn_stop)
        self.direction_group.setLayout(direction_layout)

        speed_group = QGroupBox("Contrôle de Vitesse")
        speed_layout = QVBoxLayout()

        self.speed_label = QLabel("Vitesse (0-100%)")
        self.speed_slider = QComboBox()
        self.speed_slider.addItems([str(i) for i in range(0, 101, 10)])
        self.speed_slider.setCurrentIndex(5)
        self.speed_slider.currentTextChanged.connect(self.on_speed_changed)

        speed_layout.addWidget(self.speed_label)
        speed_layout.addWidget(self.speed_slider)
        speed_group.setLayout(speed_layout)

        self.btn_send = QPushButton("Envoyer Message Test ROS")
        self.btn_send.clicked.connect(self.onPushSend)

        control_layout.addWidget(mode_group)
        control_layout.addWidget(self.direction_group)
        control_layout.addWidget(speed_group)
        control_layout.addWidget(self.btn_send)
        control_layout.addStretch()
        control_panel.setLayout(control_layout)

        # --- Affichage ---
        display_panel = QGroupBox("Informations du Robot")
        display_layout = QVBoxLayout()

        data_group = QGroupBox("Données du Robot")
        data_layout = QVBoxLayout()
        self.wheel_speed_label = QLabel("Vitesse des roues: 0 tr/min")
        self.robot_speed_label = QLabel("Vitesse du robot: 0 m/s (0 km/h)")
        self.obstacle_label = QLabel("Obstacles: Aucun détecté")

        data_layout.addWidget(self.wheel_speed_label)
        data_layout.addWidget(self.robot_speed_label)
        data_layout.addWidget(self.obstacle_label)
        data_group.setLayout(data_layout)

        camera_group = QGroupBox("Caméra")
        camera_layout = QVBoxLayout()
        self.camera_label = QLabel()
        self.camera_label.setAlignment(Qt.AlignCenter)
        self.camera_label.setMinimumSize(640, 480)
        camera_layout.addWidget(self.camera_label)
        camera_group.setLayout(camera_layout)

        display_layout.addWidget(data_group)
        display_layout.addWidget(camera_group)
        display_panel.setLayout(display_layout)

        main_layout.addWidget(control_panel, 1)
        main_layout.addWidget(display_panel, 2)

    def on_mode_changed(self):
        if self.manual_radio.isChecked():
            self.direction_group.setEnabled(True)
            self.random_timer.stop()
            self.publisher_mode.publish(Int32(data=0))
        elif self.random_radio.isChecked():
            self.direction_group.setEnabled(False)
            self.publisher_mode.publish(Int32(data=1))
            self.random_timer.start(self.random_move_interval)
        elif self.tracking_radio.isChecked():
            self.direction_group.setEnabled(False)
            self.random_timer.stop()
            self.publisher_mode.publish(Int32(data=2))

    def random_move(self):
        directions = ["forward", "backward", "left", "right", "stop"]
        self.send_direction_cmd(random.choice(directions))

    def send_direction_cmd(self, direction):
        self.publisher_cmd.publish(String(data=direction))
        print(f"Commande envoyée: {direction}")

    def on_speed_changed(self, speed):
        self.publisher_speed.publish(Float32(data=float(speed) / 100.0))
        print(f"Vitesse changée à: {speed}%")

    def onPushSend(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_cmd.publish(msg)
        print(f'Publication: "{msg.data}"')
        self.i += 1

    def listener_callback(self, msg):
        print(f'Message reçu: "{msg.data}"')

    def wheel_speed_callback(self, msg):
        self.wheel_speed_label.setText(f"Vitesse des roues: {msg.data:.2f} tr/min")

    def robot_speed_callback(self, msg):
        speed_kph = msg.data * 3.6
        self.robot_speed_label.setText(f"Vitesse du robot: {msg.data:.2f} m/s ({speed_kph:.2f} km/h)")

    def obstacle_callback(self, msg):
        self.obstacle_label.setText(f"Obstacles: {msg.data}")

    def camera_callback(self, msg):
        try:
            img_np = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
            q_img = QImage(img_np.data, msg.width, msg.height, QImage.Format_RGB888).rgbSwapped()
            self.camera_label.setPixmap(QPixmap.fromImage(q_img).scaled(
                self.camera_label.width(), self.camera_label.height(), Qt.KeepAspectRatio))
        except Exception as e:
            print(f"Erreur affichage image: {e}")

    def onTimerTick(self):
        rclpy.spin_once(self.node, executor=None, timeout_sec=0.01)

    def closeEvent(self, event):
        self.timer.stop()
        if self.random_radio.isChecked():
            self.random_timer.stop()
        self.node.destroy_node()
        rclpy.shutdown()
        event.accept()

def main(args=None):
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())

if __name__ == '__main__':
    main()
