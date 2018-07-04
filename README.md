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

#### Solution

#### Demo

#### Source code

#### Keywords
`Tensorflow`, `Python`
