import cv2
import numpy as np

# Position de l'ellipse
Pos = (50, 50)

def base():
    # Création de l'image M (rouge)
    M = np.full((200, 100, 3), (0, 0, 255), dtype=np.uint8)  # OpenCV est en BGR
    print("M =\n", M, "\n")

    # Dessin d'une ellipse bleue
    cv2.ellipse(
        M,
        Pos,
        (5, 5),
        10,
        0,
        360,
        (255, 0, 0),  # Bleu en BGR
        2,
        cv2.LINE_8
    )

    # Affichage de M
    cv2.namedWindow("1", cv2.WINDOW_AUTOSIZE)
    cv2.imshow("1", M)

    # Création de l'image N (rose/magenta)
    N = np.full((200, 100, 3), (255, 0, 255), dtype=np.uint8)
    print("N =\n", N, "\n")

    # Affichage de N
    cv2.namedWindow("2", cv2.WINDOW_AUTOSIZE)
    cv2.imshow("2", N)

    cv2.waitKey(0)
    cv2.destroyAllWindows()
