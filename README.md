# Self-Driving Car Engineer Nanodegree
This repository contains solutions for projects of [Udacity Self-Driving Car Engineer Nanodegree](https://udacity.com/course/self-driving-car-engineer-nanodegree--nd013)
### Project 1: Finding Lane Lines on the Road
![Finding Lane Lines on the Road](p1_lane_finding/preview.gif)  
#### Problem
Detection of lane lines on a video
#### Solution
Using [Canny edge detector](https://en.wikipedia.org/wiki/Canny_edge_detector) and [Hough transform](https://en.wikipedia.org/wiki/Hough_transform) implementations from [OpenCV](https://opencv.org/) I extracted candidate lane lines from individual frames based on level of confidence and assumptions about spatial characteristics (position, angle and length). The final lane lines **in the individual frame** are weighted averages of the frame's candidate lines (longer candidates have larger weights). Resulting lane lines **on the video** are averages of the final lane lines from current and several previous individual frames. All variables were manually tuned  
#### Demo
Simple Case - https://youtu.be/En4_FAs5c-s  
More Advanced Case - https://youtu.be/5emFX8R4zpA  
Challenge - https://youtu.be/U8C0otDC1F8
#### Source code
[p1_lane_finding](p1_lane_finding)  
#### Keywords
`Python`, `Computer Vision`, `OpenCV`

### Project 2: Traffic Sign Classification
![Finding Lane Lines on the Road](p2_traffic_sign_classifier/preview.png)  
#### Problem
Classification of traffic signs
#### Solution
After some initial exploration of the dataset I augmented it by rotating images. I didn't convert to grayscale to keep color data. Not only it increased the size of training data, but also in real life traffic signs can be observed at some angle depending on relative position of the car and signs. LeNet architecture worked pretty well on augmented data. I only had to modify input and output dimensions to fit dataset. Then the model was tested on traffic signs found on the internet.
#### Dataset
http://benchmark.ini.rub.de/?section=gtsrb&subsection=dataset
#### Source code
[p2_traffic_sign_classifier](p2_traffic_sign_classifier)
#### Keywords
`TensorFlow`, `Deep Learning`, `LeNet`
