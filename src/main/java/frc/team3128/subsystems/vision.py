import cv2
import numpy as np

# runPipeline() is called every frame by Limelight's backend.
# llrobot is sent via networktables to the python snapscript.
def runPipeline(image, llrobot):
    # convert the input image to the HSV color space
    img_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    purple_low = np.array([130,50,50])  
    purple_high = np.array([160,255,255]) 
    img_threshold = cv2.inRange(img_hsv, purple_low, purple_high)
    
    contoursp, hierarchy = cv2.findContours(img_threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    llpython = [0,0,0,0,0,0,0,0]
    largestContour = np.array([[]])
    
    if len(contoursp) > 0:
        largestContour = max(contoursp, key=cv2.contourArea)
        
        if cv2.contourArea(largestContour) > 250:  
            cv2.drawContours(image, [largestContour], -1, (224, 227, 39), 1)
            x,y,w,h = cv2.boundingRect(largestContour)
            llpython = [0,x,y,w,h,9,8,7]
    
    return largestContour, img_threshold, llpython # llpython sent to java code via nt if needed