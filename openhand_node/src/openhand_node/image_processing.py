import cv2
import numpy as np
from vsp.detector import CvBlobDetector, optimize_blob_detector_params
import os

'''
What to include in this file:
    Image processing - Gaussian adaptive thresholding, cropping, blob detection and masking
    After processing applied: work out SSIM and update the spreadsheet from data collection.

'''

thumb_crop = [100,0,215,240]
middle_crop = [100,0,215,240]
thresh_width = 35
thresh_offset = -31


def crop_image(image, crop):
    x0,y0,x1,y1 = crop
    frame = image[y0:y1,x0:x1]
    return frame

def load_frames():
    files = os.listdir('images/Middle')
    frames = []
    for f in files:
        img = cv2.imread('images/Middle/'+f)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)# convert to grayscale
        img = crop_image(img, middle_crop)
        frames.append(img)
        
    return np.array(frames)




def main():
    image = cv2.imread('images/Middle/default.jpg')
    print(image.shape)

    cv2.imshow('Raw Image', image)
    cv2.waitKey()

    # Make grayscale
    image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)# convert to grayscale
    cv2.imshow('Greyscale Image', image)
    cv2.waitKey()

    # Apply the crop
    image = crop_image(image, thumb_crop)
    cv2.imshow('Cropped Image', image)
    cv2.waitKey()

    # Gaussian Threshold
    thresh_image = cv2.adaptiveThreshold(image, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, thresh_width, thresh_offset)
    cv2.imshow('Thresholded Image', thresh_image)
    cv2.waitKey()

    # Init blob detector
    frames = load_frames()
    print(frames.shape)
    #params = optimize_blob_detector_params(frames,
                                           #target_blobs=30,
                                           #min_threshold_range=(0, 300),
                                           #max_threshold_range=(0, 300),
                                           #min_area_range=(0, 200),
                                           #max_area_range=(0, 200),
                                           #min_circularity_range=(0.1, 0.9),
                                           #min_inertia_ratio_range=(0.1, 0.9),
                                           #min_convexity_range=(0.1, 0.9),
                                           #)
    params = {'min_threshold': 77.5368495403626, 'max_threshold': 249.02793620380038, 'filter_by_color': True, 'blob_color': 255, 'filter_by_area': True, 'min_area': 35.55149885207927, 'max_area': 162.7512498629937, 'filter_by_circularity': True, 'min_circularity': 0.5804456114609264, 'filter_by_inertia': True, 'min_inertia_ratio': 0.6160544249261788, 'filter_by_convexity': True, 'min_convexity': 0.5330051459398885}
    print(params)
    det = CvBlobDetector(**params) 

    # blob detection on unthresholded
    keypoints = det.detect(image)
    print(keypoints)
    kpts = [cv2.KeyPoint(kp.point[0], kp.point[1], kp.size) for kp in keypoints]
    # Draw detected blobs as red circles.
    # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
    im_with_keypoints = cv2.drawKeypoints(image, kpts, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    # Show keypoints
    cv2.imshow("Unthresh Blobs", im_with_keypoints)
    cv2.waitKey()


    # blob detected on thresholded image
    keypoints = det.detect(thresh_image)
    kpts = [cv2.KeyPoint(kp.point[0], kp.point[1], kp.size) for kp in keypoints]
    # Draw detected blobs as red circles.
    # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
    im_with_keypoints = cv2.drawKeypoints(thresh_image, kpts, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    # Show keypoints
    cv2.imshow("Thresh Blobs", im_with_keypoints)
    cv2.waitKey()






if __name__ == '__main__':
    main()