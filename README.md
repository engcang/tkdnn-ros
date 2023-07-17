# tkDNN-ROS
+ `YOLO` object detection with `ROS` and `TensorRT` using [`tkDNN`](https://github.com/ceccocats/tkDNN)
	+ Currently, only `YOLO` is supported.
	+ (Optional) Image rectification is added - supports `pinhole-radtan`, `pinhole-equidistant` (Fisheye)
    + (Optional) Downsampled inference is added (e.g. image is coming in at 30Hz but infer it upto 15Hz)
    + (Optional) Image saving is added - save only detected image in `.jpg` format in `image` folder

<br>

### Comparison of performance and other `YOLO` implementation? - [here](https://github.com/engcang/ros-yolo-sort/tree/master/YOLO_and_ROS_ver)


<br>

## Dependencies
+ `CUDA`, `cuDNN`, `TensorRT` (version **8**)
+ `.rt` file, built with `.weights` and `.cfg` files
<details><summary>[Click to see how to make `.rt` file in detail]</summary>

### ● prepare `.rt file` ★much work to do★
+ Export `weight` and `cfg` file into `.bin` files as [original repo](https://github.com/ceccocats/tkDNN#how-to-export-weights)
~~~shell
Get the darknet (only for export, not used for detection)
$ git clone https://git.hipert.unimore.it/fgatti/darknet.git
If this darknet repo does not work, try with this one: 
              https://github.com/AlexeyAB/darknet/issues/6116#issuecomment-655483646
              https://github.com/AlexeyAB/darknet/files/4890564/darknet-master.zip
$ cd darknet
$ make
$ mkdir layers debug
$ ./darknet export <path-to-cfg-file> <path-to-weights> layers
-> .bin files are generated in debug and layers folders
~~~

+ Build `.rt` file, which can generate `executable file`
~~~shell
$ cd tkDNN/tests/darknet
$ cp yolo4.cpp <name_you_want>.cpp
$ gedit <name_you_want>.cpp
~~~
~~~cpp
std::string bin_path = "path from tkDNN/build folder"; //edit

// Edit here with output layer, check exported 'layers' folder
// e.g., for yolo v4 tiny, exported .bin file with name 'g' are 'g30.bin' and 'g37.bin'
// files starting with 'g' are output layer
std::vector<std::string> output_bins = {
    bin_path + "/debug/layer30_out.bin",
    bin_path + "/debug/layer37_out.bin"
};

// also check .cfg and .names (.txt) files directory
std::string cfg_path  = std::string(TKDNN_PATH) + "/tests/darknet/cfg/yolo4tiny.cfg";
std::string name_path = std::string(TKDNN_PATH) + "/tests/darknet/names/coco.names";
~~~
~~~shell
$ cd tkdnn/build
$ cmake .. && make

# executable file name with <name_you_want> is generated.
# Excute it to generate .rt file
### it reads the .cfg and .bin files written in <name_you_want>.cpp file, so directories should be accurate
$ ./test_<name_you_want> 
~~~

### ● Execution, refer [here](https://github.com/ceccocats/tkDNN/blob/master/docs/demo.md) for more detail
~~~shell
$ cd tkdnn/build

Edit the paths in the demoConfig.yaml file before!
$ ./demo ../demo/demoConfig.yaml
~~~

### ● Changing inference data type: **re-generate `.rt file` after export tkdnn mode**
~~~shell
type one of belows: (TKDNN_MODE=FP32 is default before change)
$ export TKDNN_MODE=FP16
$ export TKDNN_MODE=INT8

and re-generate .rt file as above before execute.
~~~

---

</details>


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

+ Make sure you have `.rt` files with your `CUDA`, `cuDNN`, `TensorRT` version.

+ change parameters in `main.launch` file

+ run the code
~~~shell
$ roslaunch tkdnn-ros main.launch
~~~
