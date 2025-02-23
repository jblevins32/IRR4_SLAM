import numpy as np
import cv2

def capture(frame):
    '''
    This function currently detects the largest orange circle in a frame using openCV
    '''
    # convert to HSV
    np_arr = np.frombuffer(frame.data, np.uint8)
    frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    img_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Process the image
    img_orange = image_process(img_hsv)

    # Track the ball
    img_orange_tracked, frame_tracked, circle1 = track(img_orange, frame)
    
    # If no circle is found, set the location to center so the robot does not turn at all
    if circle1 is None:
        circle1 = [320/2]

    # Display the resulting frame
    cv2.imshow('Original',img_orange_tracked)
    cv2.imshow('Processed',frame_tracked)
    key = cv2.waitKey(1) & 0xFF
    
    # Center the circle coordinates to the center of the image and norm it so the coordinate range is in (-1,1). Negative values are left of center and positive values are right of center
    img_width = 320 # horizontal pixels
    obj_coords = (img_width/2 - circle1[0])/(img_width/2)

    return obj_coords
    # When everything done, release the capture
    # cv2.destroyAllWindows()
    
def image_process(img_hsv):
    # Process the image by focusing on the orange color
    lower = np.array([0,50,50])
    higher = np.array([40,255,255])
    img_orange = cv2.inRange(img_hsv, lower, higher)
    
    # Add gaussian blur
    img_orange = cv2.GaussianBlur(img_orange,ksize=(35,35),sigmaX=0)
    
    return img_orange
    
def track(img, img_original):
    # Get the circles from the image. Output is N circles with x,y,r info
    circles = cv2.HoughCircles(img, cv2.HOUGH_GRADIENT, 1, 90, param1=100, param2=30, minRadius=10, maxRadius=500)
   
    # Only show the circles if they are detected, else None
    if circles is not None:
        # Only look at largest circle
        circle1 = circles[0,np.argmax(circles[0,:,-1], axis=0)]
        print(circle1)
        circle1 = np.uint16(np.around(circle1))
        
        # Look for all the circles and add them to the images
        cv2.circle(img,(circle1[0],circle1[1]),circle1[2],(0,255,0),2)
        cv2.circle(img_original,(circle1[0],circle1[1]),circle1[2],(0,255,0),2)
    else:
        print('No circles found')
        circle1 = None

        
    return img, img_original, circle1

if __name__ == "__main__":
    capture()