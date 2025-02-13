import numpy as np
import cv2 as cv
from matplotlib import pyplot as plt
# import rospy2 as ros

# Function to return the number of white pixels from a given image
# INPUT: unprocesse img file
# OUTPUT: num of white px

def num_white_pixels(img):
    assert img is not None, "file could not be read, check with os.path.exists()"
 
    # Otsu's thresholding after Gaussian filtering
    blur = cv.GaussianBlur(img,(5,5),0)
    # Convert to white or black
    _, img_wb = cv.threshold(blur,127,255,0)
    
    white_px = np.sum(img_wb==255)

    #################################
    # DEBUG:
    # plt.imshow(img_wb, cmap="gray")
    # plt.show()
    # print(img_wb)
    # print(white_px)
    #################################
    print("[white_pixels] {} white pixels found".format(white_px))
    return white_px


if __name__ == "__main__":
    try:
        img = cv.imread(r"C:\Users\Flyte\OneDrive - Delft University of Technology\Subjects\Microsat Engineering\code\imgs\testimg.jpg", cv.IMREAD_GRAYSCALE)  #queryimage # left image
        num_white_pixels(img)
    except:
        print("[white_pixels]: error")
    # except ros.ROSInterruptException:
    #     pass