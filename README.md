# SFND 2D Feature Tracking

<img src="images/keypoints.png" width="820" height="248" />

The idea of the camera course is to build a collision detection system - that's the overall goal for the Final Project. As a preparation for this, you will now build the feature tracking part and test various detector / descriptor combinations to see which ones perform best. This mid-term project consists of four parts:

* First, you will focus on loading images, setting up data structures and putting everything into a ring buffer to optimize memory load. 
* Then, we will integrate several keypoint detectors such as HARRIS, SHITOMASI, AKAZE, ORB, FAST, BRISK and SIFT and compare them with regard to number of keypoints and speed. 
* In the next part, we will then focus on descriptor extraction using AKAZE, ORB, FREAK, BRIEF, BRISK and SIFT diescriptors and matching using brute force and also the FLANN approach.
* In the last part, once the code framework is complete, we will test the various algorithms in different combinations and compare them with regard to some performance measures to select the best 3 algorithms for our task. 
 

## Dependencies for Running Locally
* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* OpenCV >= 4.1
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./2D_feature_tracking`.

# Benchmark
The example matching result is:
<img src="images/example.png" width="820" height="180" />

This creates a `src/results.csv` file that show all different combinations with all keypoints, all matches, and all time for every combination (detector + descriptor).

I used  KNN match selection (k=2) and performed descriptor distance ratio filtering with t=0.8 in file `matching2D.cpp`.

|Sr. No. | Detector + Descriptor |Total Keypoints |Total Matches |Total Time (ms) |
|:---:|:---:|:----:|:-----:|:-----:|
| 1 | SHITOMASI + BRISK |1259 |768 |11.2766 |
| 2 | SHITOMASI + BRIEF |1259 |961 |2.62141 |
| 3 | SHITOMASI + ORB |1259 |915 |2.85638 |
| 4 | SHITOMASI + FREAK |1259 |786 |3.47913 |
| 5 | SHITOMASI + SIFT |1259 |961 |3.97998 |
| 6 | SHITOMASI + AKAZE |1259 |N/A |N/A |
| 7 | HARRIS + BRISK |248 |142 |12.0708 |
| 8 | HARRIS + BRIEF |248 |173 |2.96121 |
| 9 | HARRIS + ORB |248 |160 |3.29868 |
| 10 | HARRIS + FREAK |248 |146 |4.31864 |
| 11 | HARRIS + SIFT |248 |163 |4.07526 |
| 12 | HARRIS + AKAZE |248 |N/A |N/A |
| 13 | BRISK + BRISK |2737 |1548 |19.0391 |
| 14 | BRISK + BRIEF |2737 |1678 |11.6092 |
| 15 | BRISK + ORB |2737 |1482 |13.0184 |
| 16 | BRISK + FREAK |2737 |1502 |12.5579 |
| 17 | BRISK + SIFT |2737 |1629 |16.2214 |
| 18 | BRISK + AKAZE |2737 |N/A |N/A |
| 19 | FAST + BRISK |1535 |900 |9.64457 |
| 20 | FAST + BRIEF |1535 |1110 |0.814345 |
| 21 | FAST + ORB |1535 |1091 |1.11988 |
| 22 | FAST + FREAK |1535 |893 |1.88817 |
| 23 | FAST + SIFT |1535 |1070 |2.63134 |
| 24 | FAST + AKAZE |1535 |N/A |N/A |
| 25 | SIFT + BRISK |1524 |602 |18.2577 |
| 26 | SIFT + BRIEF |1524 |713 |8.35852 |
| 27 | SIFT + ORB |1524 |713 |8.35852 |
| 28 | SIFT + FREAK |1524 |602 |9.86334 |
| 29 | SIFT + SIFT |1524 |840 |14.0014 |
| 30 | SIFT + AKAZE |1524 |N/A |N/A |
| 31 | ORB + BRISK |1150 |744 |20.5978 |
| 32 | ORB + BRIEF |1150 |540 |2.35482 |
| 33 | ORB + ORB |1150 |754 |4.4757 |
| 34 | ORB + FREAK |1150 |417 |3.22363 |
| 35 | ORB + SIFT |1150 |756 |7.20027 |
| 36 | ORB + AKAZE |1150 |N/A |N/A |
| 37 | AKAZE + BRISK |1670 |1209 |16.3556 |
| 38 | AKAZE + BRIEF |1670 |1271 |8.55243 |
| 39 | AKAZE + ORB |1670 |1185 |9.68565 |
| 40 | AKAZE + FREAK |1670 |1191 |9.27461 |
| 41 | AKAZE + SIFT |1670 |1273 |10.3741 |
| 42 | AKAZE + AKAZE |1670 |1258 |15.6027 |

## Top 3 detector/ descriptor pairs

I calculated the best algorithm by dividing the all Time for one combination (detector+descriptor) by the all matches(only for considerable big numbers)
and took the most 3 least results.

|Sr. No. | Detector + Descriptor |Total Keypoints |Total Matches |Total Time (ms) |
|:---:|:---:|:----:|:-----:|:-----:|
|1 | FAST + BRIEF |1535 |1110 |0.814345 |
|2 | AKAZE + BRIEF |1670 |1271 |8.55243 |
|3 | BRISK + BRIEF |2737 |1678 |11.6092 |