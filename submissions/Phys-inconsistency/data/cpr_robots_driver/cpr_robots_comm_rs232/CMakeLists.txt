cmake_minimum_required(VERSION 2.8.3)
project(cpr_robots_comm_rs232)

find_package(catkin REQUIRED COMPONENTS roscpp roslib)
find_package(Boost REQUIRED COMPONENTS filesystem thread)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES cpr_robots_comm_rs232
   CATKIN_DEPENDS roscpp roslib
   DEPENDS Boost
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

## Declare a library
add_library(cpr_robots_comm_rs232
   src/cpr_robots_comm_rs232.cpp
)
target_link_libraries(cpr_robots_comm_rs232
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
)

add_executable(test_serial src/test_serial.cpp)
target_link_libraries(test_serial
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
   cpr_robots_comm_rs232
)

