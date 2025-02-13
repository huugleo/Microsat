import cv2
import numpy as np
def find_center(img): 
    # convert image to grayscale image
    gray_image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # convert the grayscale image to binary image
    ret,bin_img = cv2.threshold(gray_image,127,255,0)

    # calculate moments of binary image
    M = cv2.moments(bin_img)
    
    # calculate x,y coordinate of center
    centroid_x = int(M["m10"] / M["m00"])
    centroid_y = int(M["m01"] / M["m00"])
    
    # put text and highlight the center
    cv2.circle(bin_img, (centroid_x, centroid_y), 5, (0, 0, 0), -1)
    
    # display the image
    cv2.imshow("Image", bin_img)
    cv2.waitKey(0)
    
    # find x and y coordinate of center picture
    dimensions = bin_img.shape
    img_center_x = dimensions[1]/2
    img_center_y = dimensions[0]/2
    print(centroid_x, img_center_x)
    print(centroid_y, img_center_y)
    # calculate distance between centroid and center image
    x_distance = centroid_x-img_center_x
    y_distance = centroid_y-img_center_y
    return x_distance, y_distance

if __name__ == '__main__':
    img = cv2.imread("test_picture.jpg")
    img_resize = cv2.resize(img, (960,540))
    img_invert = cv2.bitwise_not(img_resize)
    find_center(img_invert)