# project name
PROJECT(camera_data_processing)
# requirement of cmake version
cmake_minimum_required(VERSION 3.5)

# set the directory of executable files
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${opencv_test_SOURCE_DIR}/bin)

# find required opencv
find_package(OpenCV REQUIRED)
# directory of opencv headers
include_directories(${OpenCV_INCLUDE_DIRS})

# name of executable file and path of source file
add_executable(camera_data_processing
    src/main.cpp
    src/Lane.cpp
    src/CameraModel.cpp
    src/Detection.cpp
    src/Utility.cpp
    src/RScamera.cpp
    src/Disparity.cpp
    src/Motion.cpp
    src/Visualization.cpp
    src/GroundFilter.cpp
    src/ObjectFilter.cpp
    src/ObjectDetector.cpp
    src/CameraObjectDetection.cpp
    src/MonoDetector.cpp
    src/ELAS/descriptor.cpp
    src/ELAS/elas.cpp
    src/ELAS/filter.cpp
    src/ELAS/matrix.cpp
    src/ELAS/triangle.cpp
)


# include files
INCLUDE_DIRECTORIES(SYSTEM ${CMAKE_CURRENT_SOURCE_DIR}/include)

# directory of opencv library
link_directories(${OpenCV_LIBRARY_DIRS})


# libraries
target_link_libraries(
    realsense_cv
    ${OpenCV_LIBS} 
    realsense2
)

