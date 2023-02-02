import cv2

'''
What to include in this file:
    Image processing - Gaussian adaptive thresholding, cropping, blob detection and masking
    After processing applied: work out SSIM and update the spreadsheet from data collection.

'''

thumb_crop = [110,0,225,240]

def crop_image(image, crop):
    x0,y0,x1,y1 = crop
    frame = image[y0:y1,x0:x1]
    return frame



def main():
    image = cv2.imread('images/Thumb/default.jpg')

    cv2.imshow('Raw Image', image)
    cv2.waitKey()

    # Apply the crop
    image = crop_image(image, thumb_crop)
    cv2.imshow('Cropped Image', image)
    cv2.waitKey()






if __name__ == '__main__':
    main()