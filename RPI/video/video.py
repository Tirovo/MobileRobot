import cv2
import numpy as np
import sys
import threading

import rclpy
sys.path.append('/home/rpi/ros2_ws/src/py_pubsub/py_pubsub')
from publisher_member_function import MinimalPublisher


def red_detect(frame):
    nb_pixels = 0
    object_pos = (-1, -1)

    # Conversion BGR → HSV
    frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Détection du rouge (deux intervalles car le rouge est sur les extrémités du cercle HSV)
    lower_red1 = np.array([0, 135, 135])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([159, 135, 135])
    upper_red2 = np.array([179, 255, 255])

    mask1 = cv2.inRange(frame_hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(frame_hsv, lower_red2, upper_red2)
    mask = cv2.bitwise_or(mask1, mask2)

    # Recherche du barycentre
    moments = cv2.moments(mask)
    if moments["m00"] > 0:
        cx = int(moments["m10"] / moments["m00"])
        cy = int(moments["m01"] / moments["m00"])
        object_pos = (cx, cy)
        nb_pixels = int(moments["m00"])

        # Dessin d'une ellipse centrée sur le barycentre
        cv2.ellipse(frame, object_pos, (10, 10), 0, 0, 360, (255, 0, 0), 2)

    # Affichage
    cv2.imshow("Capture - Red Detection", frame)
    cv2.imshow("Red Mask", mask)

    return object_pos


def sharpen(my_image):
    # Vérifie que l’image est bien en 8 bits
    if my_image.dtype != np.uint8:
        raise ValueError("Image must be 8-bit")

    result = np.zeros_like(my_image)
    result[1:-1, 1:-1] = cv2.saturate_cast(
        5 * my_image[1:-1, 1:-1]
        - my_image[1:-1, 0:-2]
        - my_image[1:-1, 2:]
        - my_image[0:-2, 1:-1]
        - my_image[2:, 1:-1]
    )

    # Bords à zéro
    result[0, :] = 0
    result[-1, :] = 0
    result[:, 0] = 0
    result[:, -1] = 0

    cv2.imshow("Sharpen", result)


def display_video():
    cap = cv2.VideoCapture(0)  # Webcam 0

    rclpy.init()
    publisher = MinimalPublisher()

    # Lancer le spin ROS2 dans un thread à part
    spin_thread = threading.Thread(target=rclpy.spin, args=(publisher,), daemon=True)
    spin_thread.start()

    if not cap.isOpened():
        print("Erreur : caméra non détectée.")
        return -1

    while True:
        ret, frame = cap.read()
        if not ret:
            print(" --(!) No captured frame -- Break!")
            break

        barycentre = red_detect(frame)
        publisher.send(barycentre[0], barycentre[1])

        if cv2.waitKey(10) & 0xFF == ord('c'):
            break

    cap.release()
    publisher.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()
    return 0
