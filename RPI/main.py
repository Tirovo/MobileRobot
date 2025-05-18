import sys
import base.base as base
import image.image as image
import video.video as video

def main():
    print("Loading camera")

    if len(sys.argv) < 2:
        print("Not enough parameters")
        return -1

    if sys.argv[1] == "B":
        base.base()
    elif sys.argv[1] == "I":
        image.image_process(sys.argv)
    elif sys.argv[1] == "V":
        video.display_video()()
    else:
        print("!!! First parameter : B/I/V !!!")

if __name__ == "__main__":
    main()
