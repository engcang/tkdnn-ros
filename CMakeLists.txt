cmake_minimum_required(VERSION 3.4)
project(tkdnn_ros)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14") # if want to use auto in Lamda function, should be more recent than c++14
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")




find_package(tkDNN REQUIRED)
if(tkDNN_FOUND)
  message(WARNING "tkDNN FOUND, Version:" ${tkDNN_VERSION} " directory: " ${tkDNN_DIR})
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  sensor_msgs
  cv_bridge
  message_generation
  message_runtime
)

find_package(OpenCV REQUIRED)
if(OPENCV_FOUND)
  message(WARNING "OpenCV FOUND, Version:" ${OpenCV_VERSION} " directory: " ${OpenCV_INCLUDE_DIRS})
endif()




add_message_files(
  FILES
    bbox.msg
    bboxes.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs
)




catkin_package(
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp roslib sensor_msgs std_msgs cv_bridge message_generation message_runtime
)


###########
## Build ##
###########

include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/tkDNN/installed/include/tkDNN
  ${tkDNN_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library( tkDNN SHARED IMPORTED)
set_target_properties(tkDNN PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/tkDNN/installed/lib/libtkDNN.so)
add_library( kernels SHARED IMPORTED)
set_target_properties(kernels PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/tkDNN/installed/lib/libkernels.so)

add_executable(${PROJECT_NAME}_node src/main.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${OpenCV_LIBS} ${tkDNN_LIBRARIES} tkDNN kernels)
