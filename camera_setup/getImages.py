import cv2
def get_imgs():

    cap = cv2.VideoCapture(1)

    num = 0

    while cap.isOpened():

        succes, img = cap.read()

        k = cv2.waitKey(5)

        if k == 27:
            break
        elif k == ord('s'): # wait for 's' key to save and exit
            cv2.imwrite('camera_setup\cal_images\img' + str(num) + '.png', img)
            print("image saved!")
            num += 1

        cv2.imshow('Img 1',img)
get_imgs()