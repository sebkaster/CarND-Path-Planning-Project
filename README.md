# CarND-Path-Planning-Project
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

<!-- TABLE OF CONTENTS -->
## Table of Contents

* [About the Project](#about-the-project)
* [Getting Started](#getting-started)
  * [Prerequisites](#prerequisites)
  * [Installation](#installation)
* [Structure](#structure)
* [Simulator Description](#simulator-description)
* [Reflection](#usage)
* [Contributing](#contributing)
* [License](#license)
* [Contact](#contact)


[//]: # (Image References)

[image1]: ./imgs/project-pic.PNG


About the Project
---

In this project the goal is to safely navigate around a virtual highway with other traffic participants. Furthermore, it is desired that jerk is minimized and other traffic participants are not endangered.

![alt text][image1]


<!-- GETTING STARTED -->
## Getting Started

The software is written in C++17 and tested on Linux.

### Prerequisites

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 9.0.0
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

### Build

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

<!-- STRUCTURE -->
## Structure
The directory structure of this repository is as follows:
```
root
|   install-mac.sh
|   install-ubuntu.sh
|   cmakepatch.txt
|   CMakeLists.txt
|   README.md
|   set_git.sh
|   LICENSE
|
|___data
|   |   
|   |   highway_map.csv
|
|___imgs
|   |   
|   |   project_pic.png 
|   
|___src
    |   Eigen-3.3-Library
    |   car.h
    |   car.cpp
    |   constants.h
    |   helpers.h
    |   json.hpp
    |   spline.h
    |   main.cpp
```

## Simulator Description
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

<!-- REFLECTION -->
## Reflection

Based on the provided code, the trajectory generation is divided into three steps:

#### [Prediction](./src/main.cpp#L54)

In order to plan a safe trajectory, we have to know where other traffic participants could be in the future. Therefore,
based on the current state of these traffic participants, the set of possible future states is calculated.

The closest travelled distance is calculated with maximum deceleration of 10 m/s^2 and the furthest travelled distance with a maximum acceleration of 10m/s^2 respectively. For these values we use a maximum velocity of 60 mph and minimum velcoity of 0 mph.
Moreover, it is assumed that the cars stay in their lane for this time horizon of 0.5 seconds.

The closest and furthest travelled distance span a region which is possibly occupied by a traffic participants.

#### [Behaviour Planner](./src/car.cpp#L76)

Based on the predictions and the current state of the ego vehicle a high-level action is determined. The ego vehicle
prioritizes to drive in its current lane but changes lane if the lane to the left or right has more free space in driving direction.

Since we do not want to endanger other traffic participants we keep a safe distance of `s = 0.55 * v`. Where `v` is the velocity in km/h. This is a rule of thumb by the german traffic law.

#### [Trajectory Generation](./src/car.cpp#L252)

This part does the calculation of the trajectory based on the speed and the result from the behavior planner, car coordinates and past path points.

To make the work less complicated, coordinates are transformed to local car coordinates. Based on the result of the behaviour planner a path is planned in frenet coordinates with four supportive points.
This frenet path is converted to local x- and y-coordinates. To generate a smooth trajectory spline is used. Based on this spline points are sampled in in a distance of 0.02 seconds. Moreover we perform acceleration and deceleration. Usually we are trying to accelerate until we have reached the speed limit of the lane of 50 mph. Nevertheless, if a car is dangerously close in front of the ego vehicle we decelerate. 


<!-- CONTRIBUTING -->
## Contributing

Contributions are what make the open source community such an amazing place to be learn, inspire, and create. Any contributions you make are **greatly appreciated**.

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

Please stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

<!-- LICENSE -->
## License

Distributed under the MIT License. Further information can be found in the _LICENSE_ file.

<!-- CONTACT -->
## Contact

Sebastian Kaster - sebastiankaster@googlemail.com

Project Link: [https://github.com/sebkaster/CarND-Path-Planning-Project](https://github.com/sebkaster/CarND-Path-Planning-Project)

