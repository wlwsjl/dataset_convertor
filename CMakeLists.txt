cmake_minimum_required(VERSION 2.8.12)
project(dataset_convertor)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

add_definitions(-std=c++11)

find_package(Eigen3 REQUIRED)
find_package(OpenCV 3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rosbag
    geometry_msgs
    nav_msgs
    sensor_msgs
    cv_bridge
    image_transport
    message_generation
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

catkin_package(
    CATKIN_DEPENDS roscpp rosbag geometry_msgs nav_msgs sensor_msgs cv_bridge image_transport message_runtime
    DEPENDS EIGEN3
)

###########
## Build ##
###########

add_executable(data_pub  src/data_pub.cpp)
target_link_libraries(data_pub ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${EIGEN_LIBS})

