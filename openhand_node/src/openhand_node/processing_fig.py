import cv2




img = cv2.imread('images/Right/Right_0.3.jpg')




frame = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)# convert to grayscale
# gaussian thresholding
frame = cv2.adaptiveThreshold(frame, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 121, -8)
# Crop the frame to only show the pins in tactip
x0,y0,x1,y1 = [110,0,225,240]
frame = frame[y0:y1,x0:x1]

cv2.imshow('test',frame)

cv2.imwrite('processed_image.jpg', frame)