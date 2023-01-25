import cv2


def process_frame(img):
    frame = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)# convert to grayscale
    # gaussian thresholding
    frame = cv2.adaptiveThreshold(frame, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 121, -8)
    # Crop the frame to only show the pins in tactip
    x0,y0,x1,y1 = [110,0,225,240]
    frame = frame[y0:y1,x0:x1]

    return frame




vid = cv2.VideoCapture(0)
vid.set(cv2.CAP_PROP_FPS,40)
vid.set(cv2.CAP_PROP_FRAME_WIDTH,320)
vid.set(cv2.CAP_PROP_FRAME_HEIGHT,240)

if not vid.isOpened():
    print("Cannot open camera")
    exit()

while True:
    ret, frame = vid.read()

    print(ret)
    cv2.imshow('frame',process_frame(frame))

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
  

# Add processing to find good params for these tactips./


# After the loop release the cap object
vid.release()
# Destroy all the windows
cv2.destroyAllWindows()