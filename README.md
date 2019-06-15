# Extended Kalman Filter Project Starter Code
Self-Driving Car Engineer Nanodegree Program

In this project, I implemented a Kalman Filter to estimate the state of a moving car. This
car has two sensors, a Laser and a Radar, and both measurements are noisy. The objective
is to use a Kalman Filter to combine the two measurements and have an improved measurement
that is more accurate.

## Implementation

The implementation involved creating the equations for a Kalman Filter and a Extended
Kalman filter (available [here](src/kalman_filter.cpp)). The setup of the Kalman Filter, initialization,
prediction step and update step can be found [here](src/FusionEKF.cpp). There is also a file with
common tools used on the code [here](src/tools.cpp).

To comply with the requirement of the project, I tried to make the code as efficient as possible, and also avoid
type conversions. I replaced all float references to double, to avoid precision loss between calculations. In my experimentation,
I noticed that this created a lot of impact, making my error reduce 50% during validation phase.


## Results


Here you can check out a video of the implementation:

[![Self driving](http://img.youtube.com/vi/fw_6EaQePjw/0.jpg)](http://www.youtube.com/watch?v=fw_6EaQePjw)

On final time step, we can see that the error is below the error requirements.


## Try it yourself

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Please see the uWebSocketIO Starter Guide page in the classroom within the EKF Project lesson for the required version and installation scripts.

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ExtendedKF

---

## Other Important Dependencies

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
