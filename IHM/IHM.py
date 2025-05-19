#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import sys
import random

from PyQt5 import QtCore
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QLabel, 
                            QLineEdit, QPushButton, QVBoxLayout, QHBoxLayout,
                            QGroupBox, QRadioButton, QComboBox)
from PyQt5.QtGui import QImage, QPixmap

class MainWindow(QMainWindow):
    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)
        
        # Initialisation de la fenÃªtre principale
        self.setMinimumSize(800, 600)  # Taille minimale plus grande pour accommoder toutes les fonctionnalitÃ©s
        self.setWindowTitle("Interface de Commande ROS2 pour Robot Mobile")
        
        # Variables pour ROS2
        self.i = 0
        self.cv_image = None
        self.bridge = CvBridge()
        
        # Initialisation ROS2
        rclpy.init(args=None) 
        self.node = Node('py_ihm_node')
        
        # Configuration des publishers ROS2
        self.publisher_cmd = self.node.create_publisher(String, 'pyqt_topic_send', 10)
        self.publisher_mode = self.node.create_publisher(Int32, 'robot_mode', 10)
        self.publisher_speed = self.node.create_publisher(Float32, 'robot_speed', 10)
        
        # Configuration des subscribers ROS2
        self.subscription_cmd = self.node.create_subscription(
            String, 'pyqt_topic_rec', self.listener_callback, 10)
        
        self.subscription_wheel_speed = self.node.create_subscription(
            Float32, 'wheel_speed', self.wheel_speed_callback, 10)
        
        self.subscription_robot_speed = self.node.create_subscription(
            Float32, 'robot_speed_feedback', self.robot_speed_callback, 10)
        
        self.subscription_obstacle = self.node.create_subscription(
            String, 'obstacle_detection', self.obstacle_callback, 10)
        
        self.subscription_camera = self.node.create_subscription(
            Image, 'camera_image', self.camera_callback, 10)
        
        # CrÃ©ation de l'interface utilisateur
        self.initUI()
        
        # Configuration du timer pour ROS2
        self.timer = QTimer()
        self.timer.timeout.connect(self.onTimerTick)   
        self.timer.start(5)  # Timer toutes les 5ms pour ROS2
        
        # Timer pour le mode alÃ©atoire
        self.random_timer = QTimer()
        self.random_timer.timeout.connect(self.random_move)
        self.random_move_interval = 3000  # Changement de direction toutes les 3 secondes
        
    def initUI(self):
        """Initialisation de l'interface utilisateur"""
        # Widget central
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # Layout principal
        main_layout = QHBoxLayout(central_widget)
        
        # Panel de contrÃ´le Ã  gauche
        control_panel = QGroupBox("ContrÃ´le du Robot")
        control_layout = QVBoxLayout()
        
        # SÃ©lection du mode de fonctionnement
        mode_group = QGroupBox("Mode de Fonctionnement")
        mode_layout = QVBoxLayout()
        
        # Boutons radio pour les modes
        self.manual_radio = QRadioButton("Mode Manuel")
        self.random_radio = QRadioButton("Mode AlÃ©atoire")
        self.tracking_radio = QRadioButton("Mode Tracking")
        
        # Configuration des boutons radio
        self.manual_radio.setChecked(True)  # Mode manuel par dÃ©faut
        self.manual_radio.toggled.connect(self.on_mode_changed)
        self.random_radio.toggled.connect(self.on_mode_changed)
        self.tracking_radio.toggled.connect(self.on_mode_changed)
        
        # Ajout des boutons au layout
        mode_layout.addWidget(self.manual_radio)
        mode_layout.addWidget(self.random_radio)
        mode_layout.addWidget(self.tracking_radio)
        mode_group.setLayout(mode_layout)
        
        # ContrÃ´les de direction (uniquement en mode manuel)
        self.direction_group = QGroupBox("Direction (Mode Manuel)")
        direction_layout = QVBoxLayout()
        
        # Boutons de direction
        self.btn_forward = QPushButton("Avancer")
        self.btn_backward = QPushButton("Reculer")
        self.btn_left = QPushButton("Tourner Ã  Gauche")
        self.btn_right = QPushButton("Tourner Ã  Droite")
        self.btn_stop = QPushButton("Stop")
        
        # Connexion des boutons
        self.btn_forward.clicked.connect(lambda: self.send_direction_cmd("forward"))
        self.btn_backward.clicked.connect(lambda: self.send_direction_cmd("backward"))
        self.btn_left.clicked.connect(lambda: self.send_direction_cmd("left"))
        self.btn_right.clicked.connect(lambda: self.send_direction_cmd("right"))
        self.btn_stop.clicked.connect(lambda: self.send_direction_cmd("stop"))
        
        # Ajout des boutons au layout
        direction_layout.addWidget(self.btn_forward)
        direction_layout.addWidget(self.btn_backward)
        direction_layout.addWidget(self.btn_left)
        direction_layout.addWidget(self.btn_right)
        direction_layout.addWidget(self.btn_stop)
        self.direction_group.setLayout(direction_layout)
        
        # ContrÃ´le de vitesse
        speed_group = QGroupBox("ContrÃ´le de Vitesse")
        speed_layout = QVBoxLayout()
        
        self.speed_label = QLabel("Vitesse (0-100%):")
        self.speed_slider = QComboBox()
        self.speed_slider.addItems([str(i) for i in range(0, 101, 10)])
        self.speed_slider.setCurrentIndex(5)  # 50% par dÃ©faut
        self.speed_slider.currentTextChanged.connect(self.on_speed_changed)
        
        speed_layout.addWidget(self.speed_label)
        speed_layout.addWidget(self.speed_slider)
        speed_group.setLayout(speed_layout)
        
        # Bouton d'envoi de message ROS (comme dans l'exemple original)
        self.btn_send = QPushButton("Envoyer Message Test ROS")
        self.btn_send.clicked.connect(self.onPushSend)
        
        # Ajout des groupes au panel de contrÃ´le
        control_layout.addWidget(mode_group)
        control_layout.addWidget(self.direction_group)
        control_layout.addWidget(speed_group)
        control_layout.addWidget(self.btn_send)
        control_layout.addStretch()
        control_panel.setLayout(control_layout)
        
        # Panel d'affichage Ã  droite
        display_panel = QGroupBox("Informations du Robot")
        display_layout = QVBoxLayout()
        
        # Affichage des donnÃ©es du robot
        data_group = QGroupBox("DonnÃ©es du Robot")
        data_layout = QVBoxLayout()
        
        self.wheel_speed_label = QLabel("Vitesse des roues: 0 tr/min")
        self.robot_speed_label = QLabel("Vitesse du robot: 0 m/s (0 km/h)")
        self.obstacle_label = QLabel("Obstacles: Aucun dÃ©tectÃ©")
        
        data_layout.addWidget(self.wheel_speed_label)
        data_layout.addWidget(self.robot_speed_label)
        data_layout.addWidget(self.obstacle_label)
        data_group.setLayout(data_layout)
        
        # Affichage de la camÃ©ra
        camera_group = QGroupBox("CamÃ©ra")
        camera_layout = QVBoxLayout()
        
        self.camera_label = QLabel()
        self.camera_label.setAlignment(Qt.AlignCenter)
        self.camera_label.setMinimumSize(640, 480)
        
        # Options d'affichage camÃ©ra
        self.camera_options = QComboBox()
        self.camera_options.addItems(["Image brute", "Image traitÃ©e"])
        self.camera_options.currentIndexChanged.connect(self.update_camera_display)
        
        camera_layout.addWidget(self.camera_options)
        camera_layout.addWidget(self.camera_label)
        camera_group.setLayout(camera_layout)
        
        # Ajout des groupes au panel d'affichage
        display_layout.addWidget(data_group)
        display_layout.addWidget(camera_group)
        display_panel.setLayout(display_layout)
        
        # Ajout des panels au layout principal
        main_layout.addWidget(control_panel, 1)
        main_layout.addWidget(display_panel, 2)
        
    def on_mode_changed(self):
        """GÃ¨re le changement de mode de fonctionnement"""
        if self.manual_radio.isChecked():
            # Mode manuel - activer les contrÃ´les de direction
            self.direction_group.setEnabled(True)
            self.random_timer.stop()
            mode_msg = Int32()
            mode_msg.data = 0  # 0 pour mode manuel
            self.publisher_mode.publish(mode_msg)
            
        elif self.random_radio.isChecked():
            # Mode alÃ©atoire - dÃ©sactiver les contrÃ´les de direction
            self.direction_group.setEnabled(False)
            mode_msg = Int32()
            mode_msg.data = 1  # 1 pour mode alÃ©atoire
            self.publisher_mode.publish(mode_msg)
            self.random_timer.start(self.random_move_interval)
            
        elif self.tracking_radio.isChecked():
            # Mode tracking - dÃ©sactiver les contrÃ´les de direction
            self.direction_group.setEnabled(False)
            self.random_timer.stop()
            mode_msg = Int32()
            mode_msg.data = 2  # 2 pour mode tracking
            self.publisher_mode.publish(mode_msg)
    
    def random_move(self):
        """GÃ©nÃ¨re un mouvement alÃ©atoire pour le mode alÃ©atoire"""
        directions = ["forward", "backward", "left", "right", "stop"]
        direction = random.choice(directions)
        self.send_direction_cmd(direction)
        
    def send_direction_cmd(self, direction):
        """Envoie une commande de direction au robot"""
        msg = String()
        msg.data = direction
        self.publisher_cmd.publish(msg)
        print(f"Commande envoyÃ©e: {direction}")
    
    def on_speed_changed(self, speed):
        """GÃ¨re le changement de vitesse"""
        speed_msg = Float32()
        speed_msg.data = float(speed) / 100.0  # Convertit en valeur entre 0 et 1
        self.publisher_speed.publish(speed_msg)
        print(f"Vitesse changÃ©e Ã : {speed}%")
    
    def onPushSend(self):
        """Envoie un message test ROS (comme dans l'exemple original)"""
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_cmd.publish(msg)
        print('Publication: "%s"' % msg.data)
        self.i += 1
        
    def listener_callback(self, msg):
        """Callback pour les messages gÃ©nÃ©raux"""
        print('Message reÃ§u: "%s"' % msg.data)
        
    def wheel_speed_callback(self, msg):
        """Callback pour la vitesse des roues"""
        speed = msg.data
        self.wheel_speed_label.setText(f"Vitesse des roues: {speed:.2f} tr/min")
        
    def robot_speed_callback(self, msg):
        """Callback pour la vitesse du robot"""
        speed_mps = msg.data
        speed_kph = speed_mps * 3.6  # Conversion m/s -> km/h
        self.robot_speed_label.setText(
            f"Vitesse du robot: {speed_mps:.2f} m/s ({speed_kph:.2f} km/h)")
        
    def obstacle_callback(self, msg):
        """Callback pour la dÃ©tection d'obstacles"""
        obstacle_info = msg.data
        self.obstacle_label.setText(f"Obstacles: {obstacle_info}")
        
    def camera_callback(self, msg):
        """Callback pour l'image de la camÃ©ra"""
        try:
            # Conversion du message ROS en image OpenCV
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.update_camera_display()
        except Exception as e:
            print(f"Erreur de traitement d'image: {e}")
    
    def update_camera_display(self):
        """Met Ã  jour l'affichage de la camÃ©ra selon l'option sÃ©lectionnÃ©e"""
        if self.cv_image is not None:
            # Applique le traitement si "Image traitÃ©e" est sÃ©lectionnÃ©e
            if self.camera_options.currentIndex() == 1:
                # Exemple de traitement: conversion en niveaux de gris
                processed_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
                # Reconvertir en BGR pour l'affichage (sinon affichage en noir et blanc)
                processed_image = cv2.cvtColor(processed_image, cv2.COLOR_GRAY2BGR)
                display_image = processed_image
            else:
                display_image = self.cv_image
            
            # Conversion de l'image OpenCV en QImage pour Qt
            height, width, channel = display_image.shape
            bytes_per_line = 3 * width
            q_img = QImage(display_image.data, width, height, bytes_per_line, QImage.Format_RGB888).rgbSwapped()
            
            # Mise Ã  jour de l'affichage
            self.camera_label.setPixmap(QPixmap.fromImage(q_img).scaled(
                self.camera_label.width(), self.camera_label.height(),
                Qt.KeepAspectRatio))
    
    def onTimerTick(self):
        """GÃ¨re les messages ROS2 Ã  intervalles rÃ©guliers"""
        rclpy.spin_once(self.node, executor=None, timeout_sec=0.01)
        
    def closeEvent(self, event):
        """Nettoyage Ã  la fermeture de la fenÃªtre"""
        self.timer.stop()
        if self.random_radio.isChecked():
            self.random_timer.stop()
        self.node.destroy_node()
        rclpy.shutdown()
        event.accept()

def main(args=None):
    """Fonction principale"""
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())

if __name__ == '__main__':
    main()