cmake_minimum_required(VERSION 2.8.3)
project(cpr_robots_hwinterface)

find_package(catkin REQUIRED COMPONENTS cpr_robots_comm_rs232 roscpp geometry_msgs tf nav_msgs nodelet)
find_package(Boost REQUIRED COMPONENTS filesystem thread)

catkin_package(
    CATKIN_DEPENDS cpr_robots_comm_rs232 roscpp geometry_msgs tf nav_msgs nodelet
    INCLUDE_DIRS include
#    DEPENDS roslib roscpp std_msgs
#    LIBRARIES 
)

include_directories(include 
${catkin_INCLUDE_DIRS}
${Boost_INCLUDE_DIRS}
)

add_library(CPRSlider src/cpr_slider.cpp)
target_link_libraries(CPRSlider ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(CPRSlider_node src/cpr_slider_node.cpp)
target_link_libraries(CPRSlider_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} CPRSlider)

add_library(CPRSlider_nodelet src/cpr_slider_nodelet.cpp)
target_link_libraries(CPRSlider_nodelet ${catkin_LIBRARIES} ${Boost_LIBRARIES} CPRSlider)
