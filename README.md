# Tennis-Ball-detection-using-OpenCV-and-ROS
OpenCV and ROS to develop ball tracking program that read video frames from a video file and using a USB camera.
## File structure and Overview
- `tennisdetection.py` - This is the main file that reads video frames and detect the tennis ball. The following operations are performed in this file:
    - Opening the video frame and capturing the RGB frame.
    - Using colour filtering to detect the tennis ball.
    - Using Contour/Edge detection techniques to detect the tennis ball.
    - Calculating the centroid of the ball and drawing a circle to detect the tennis ball.
    - Press letter `q` to quit the video frame. 

- `tennisdetection_ros.py` - This is the main file that reads video frames and detect the tennis ball using ROS. The following operations are performed in this file:
    - CVBridge is an functionality used to convert images to OpenCV compatible image from ROS compatible image.
    - Opening the video frame and capturing the RGB frame.
    - Using colour filtering to detect the tennis ball.
    - Using Contour/Edge detection techniques to detect the tennis ball.
    - Calculating the centroid of the ball and drawing a circle to detect the tennis ball.
    - Run the roscore and launch the rosnode `tennisdetection_ros.py`.
    - Press letter `q` to quit the video frame. 

## Dependencies
- Python 3.10 used.
- OpenCV Python library 
- Enable conda/other python environment and install the dependencies

## Motivation
- Learn the basic fucntionalities of OpenCV and implement them in a miniproject
