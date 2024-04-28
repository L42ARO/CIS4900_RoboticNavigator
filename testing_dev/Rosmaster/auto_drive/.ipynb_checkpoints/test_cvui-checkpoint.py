import cv2
import numpy as np
import cvui

# initial cvui
WINDOW_NAME = 'CVUI Test'
cvui.init(WINDOW_NAME)

# capture one frame
low_threshold = [5]
method = cv2.THRESH_BINARY

cap = cv2.VideoCapture(0)
if cap.isOpened():
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)    # 640
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)  # 480
    cap.set(cv2.CAP_PROP_FPS, 30)           # 30
ret, frame = cap.read()
frame = cv2.resize(frame,(int(224),int(224)))
# image size
# img_h, img_w = frame.shape[0:2]
img_h = 224
img_w = 224

# ui size
ui_h = 80
ui_w = 200

# total size
total_h = img_h+ui_h
total_w = max(ui_w, img_w)


total_frame = np.zeros((total_h,  total_w, 3), np.uint8)
total_frame[:] = (150, 150, 150)
show_text = 'Hello World!'
while(1):
    # process image
    ret, frame = cap.read()
    frame = cv2.resize(frame,(int(224),int(224)))
    total_frame[0:img_h, 0:img_w] = frame

    # draw ui
    cvui.update()
    cvui.text(total_frame, 10, img_h+15, show_text)
    
    if cvui.button(total_frame, 100, img_h+15, "Button"):
        # cvui.printf(total_frame, 90, 50, "key press")
        show_text = show_text + "123"
        

    # show
    cvui.imshow(WINDOW_NAME, total_frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()
