# Extended Kalman Filter Project Starter Code
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)
Self-Driving Car Engineer Nanodegree Program

[dataset1]: ./img/dataset1_screenshot.png
[dataset2]: ./img/dataset2_screenshot.png

## Requirements

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* Udacity Simulator [Download link](https://github.com/udacity/self-driving-car-sim/releases)


Tips for setting up the environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d).

## Compilation and Execution

1. Clone this repo
2. cd to repo root dir.
3. `mkdir build`
4. `cd build`
5. `cmake ..`
6. `make`
7. `./ExtendedKF`
8. Run simulator

---

## Rubric

### Compiling

#### Your code should compile

Code compiles without errors using `cmake` and `make`. [CMakeLists.txt]( CarND-Extended-Kalman-Filter/CMakeLists.txt ) was unchanged from the original version provided in the starter code.

### Accuracy

#### px, py, vx, vy output coordinates must have an RMSE <= [.11, .11, 0.52, 0.52] when using the file: "obj_pose-laser-radar-synthetic-input.txt which is the same data file the simulator uses for Dataset 1"

After running the algorithm against Dataset 1 and Dataset 2 in the simulator and comparing the estimated positions with the ground truth, the RMSE values are:

* Dataset 1 : [0.0973, 0.0855, 0.4513, 0.4399]
* Dataset 2 : [0.0726, 0.0965, 0.4216, 0.4932]

![dataset1][dataset1]
![dataset2][dataset2]

### Follows the Correct Algorithm

#### Your Sensor Fusion algorithm follows the general processing flow as taught in the preceding lessons.

The implementation of the Kalman filter can be found in [kalman_filter.cpp]( CarND-Extended-Kalman-Filter/src/kalman_filter.cpp). It contains the needed functions as taught in the lessons: `Predict()` for predicting the state estimate and the covariance, `Update()` to perform a normal Kalman update (LIDAR) and `UpdateEKF()` for the update in the extended case (RADAR). The Kalman filter prediction and update functions are used in the general flow of the sensor fusion algorithm which is coded in [FusionEKF.cpp](CarND-Extended-Kalman-Filter/src/FusionEKF.cpp) in `ProcessMeasurement()`.

#### Your Kalman Filter algorithm handles the first measurements appropriately.

TODO

#### Your Kalman Filter algorithm first predicts then updates.

TODO

#### Your Kalman Filter can handle radar and lidar measurements.

TODO

### Code Efficiency

#### Your algorithm should avoid unnecessary calculations.

TODO
