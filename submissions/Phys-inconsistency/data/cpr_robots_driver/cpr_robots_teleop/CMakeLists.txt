cmake_minimum_required(VERSION 2.8.3)
project(cpr_robots_teleop)

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs)
find_package(Boost REQUIRED COMPONENTS system filesystem thread)

catkin_package(
    CATKIN_DEPENDS roscpp geometry_msgs
    INCLUDE_DIRS include
#    DEPENDS roslib roscpp std_msgs
#    LIBRARIES 
)

include_directories(include 
${catkin_INCLUDE_DIRS}
${Boost_INCLUDE_DIRS}
)

add_executable(CPRTeleopKeyboard src/cpr_ctrl_loop.cpp src/cpr_teleop_keyboard.cpp)
target_link_libraries(CPRTeleopKeyboard ${catkin_LIBRARIES} ${Boost_LIBRARIES})
