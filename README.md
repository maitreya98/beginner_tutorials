# First Publisher/Subscriber ROS Package
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)

## Overview
This Respository consists of a ROS package for beginners for establishing a publisher and a subscriber node. The publisher publishes a string message onto the topic "chatter". The subscriber node subscribes to the corresponding topic. Service calls are made to the publisher to change the string the publisher publishes.

### Author
Maitreya Kulkarni

## Dependencies
The project depends on the following dependencies
* ROS-Melodic - Robotic Operating System is a meta-operating system for robots. The software can be installed from [here](http://wiki.ros.org/melodic/Installation/Ubuntu) 
* catkin - Catkin is build tool which is developed using CMake and GNU Make. You may install catkin [here](http://wiki.ros.org/catkin#Installing_catkin)

## Building and Running
Terminal:1
```
    mkdir ~/catkin_ws
    cd ~/catkin_ws
    mkdir ~/src
    cd ~/src
    git clone https://github.com/maitreya98/beginner_tutorials.git
    cd ~/catkin_ws
    catkin_make
    source devel/setup.bash
    roscore
```
Terminal: 2
```
    source devel/setup.bash
    rosrun beginner_tutorials talker
```
Terminal: 3
``` 
  source devel/setup.bash
  rosrun beginner_tutorials listener
```
## Run cppcheck
```
  cppcheck --enable=all --std=c++11 -I src/ --suppress=missingIncludeSystem $( find . -name *.cpp) > results/cppcheck_result.txt
```
# Run cpplint
```
  cpplint $( find . -name *.cpp)> results/cpplint_result.txt
```
## License
MIT License
```
Copyright (c) 2021 Maitreya Kulkarni.
Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
```


