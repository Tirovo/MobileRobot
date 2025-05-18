import cv2
import numpy as np
import sys

def negative(image):
    if len(image.shape) == 2:  # Image en niveaux de gris
        for i in range(image.shape[0]):
            for j in range(image.shape[1]):
                image[i, j] = 255 - image[i, j]
    elif len(image.shape) == 3:  # Image couleur
        for i in range(image.shape[0]):
            for j in range(image.shape[1]):
                image[i, j, 0] = 255 - image[i, j, 0]  # B
                image[i, j, 1] = 255 - image[i, j, 1]  # G
                image[i, j, 2] = 255 - image[i, j, 2]  # R

def image_process(args):
    if len(args) < 3:
        print("!!! No Image Data !!!")
        return -1

    if len(args) == 4 and args[3] == "G":
        I = cv2.imread(args[2], cv2.IMREAD_GRAYSCALE)
    else:
        I = cv2.imread(args[2], cv2.IMREAD_COLOR)

    if I is None:
        print("!!! No Image Data !!!")
        return -1

    cv2.namedWindow("Input Image", cv2.WINDOW_AUTOSIZE)
    cv2.imshow("Input Image", I)

    negative(I)

    cv2.namedWindow("Negative", cv2.WINDOW_AUTOSIZE)
    cv2.imshow("Negative", I)

    cv2.waitKey(0)
    cv2.destroyAllWindows()

    return 0
