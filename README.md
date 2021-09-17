# tkDNN-ROS
+ `YOLO` object detection with `ROS` and `TensorRT` using [`tkDNN`](https://github.com/ceccocats/tkDNN)
	+ Currently, only `YOLO` is supported.

<br>

## How to install

+ clone this repo

~~~shell
$ cd ~/<your_workspace>/src
$ git clone --recursive https://github.com/engcang/tkdnn-ros
$ cd tkdnn-ros/tkDNN
$ mkdir build 
$ mkdir installed
$ cd build
$ cmake .. -DCMAKE_BUILD_TYPE=RELEASE -DCMAKE_INSTALL_PREFIX=../installed
$ make install
~~~

+ build `ROS` package

~~~shell
$ cd ~/<your_workspace>

$ catkin build -DtkDNN_DIR=<absolute_path_to_your_workspace>/src/tkdnn-ros/tkDNN/installed/share/tkDNN/cmake

or

$ echo "export tkdnn_DIR=<absolute_path_to_your_workspace>/src/tkdnn-ros/tkDNN/installed/share/tkDNN/cmake" >> ~/.bashrc
$ . ~/.bashrc
$ catkin build

or

$ catkin config -DtkDNN_DIR=<absolute_path_to_your_workspace>/src/tkdnn-ros/tkDNN/installed/share/tkDNN/cmake
$ catkin build
~~~

+ Do not forget to register `tkDNN` libraries into `LD_LIBRARY_PATH`

~~~shell
$ cd ~/<your_workspace>
$ echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$(pwd)/src/tkdnn-ros/tkDNN/installed/lib" >> ~/.bashrc
$ . ~/.bashrc
~~~

<br>

## How to run

+ Make sure you have `.rt` files, refer [here, my other repo](https://github.com/engcang/ros-yolo-sort/tree/master/YOLO_and_ROS_ver#-tensorrttkdnn-ver-2)

+ change parameters in `main.launch` file

+ run the code
~~~shell
$ roslaunch tkdnn-ros main.launch
~~~