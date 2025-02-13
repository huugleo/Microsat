import cv2
import numpy as np

img = cv2.imread('test_picture.jpg', cv2.IMREAD_GRAYSCALE)
assert img is not None, "file could not be read, check with os.path.exists()"
imgS = cv2.resize(img, (960, 540))

 
# convert the grayscale image to binary image
ret,thresh = cv2.threshold(imgS,127,255,0)

img_inv = cv2.bitwise_not(thresh)
# calculate moments of binary image
M = cv2.moments(img_inv)
 
# calculate x,y coordinate of center
centroid_x = int(M["m10"] / M["m00"])
centroid_y = int(M["m01"] / M["m00"])
 
# put text and highlight the center
cv2.circle(img_inv, (centroid_x, centroid_y), 5, (0, 0, 0), -1)
cv2.putText(img_inv, "centroid", (centroid_x - 25, centroid_y - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
 
# display the image
cv2.imshow("Image", img_inv)
cv2.waitKey(0)
dimensions = img_inv.shape
img_center_x = dimensions[0]/2
img_center_y = dimensions[1]/2

x_distance = centroid_x-img_center_x
y_distance = centroid_y-img_center_y
print(centroid_x, img_center_x, x_distance)
print(centroid_y, img_center_y, y_distance)