cmake_minimum_required(VERSION 2.8.3)
project(al5d_pckg)
find_package(OpenCV 3.2.0 REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp actionlib actionlib_msgs message_generation message_runtime)
find_package(Boost REQUIRED COMPONENTS system)

add_message_files(
  FILES
  moveArm.msg
  calibration.msg
  position.msg
)

add_action_files(
  DIRECTORY action
  FILES communicator.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs
  message_runtime
  roscpp
  std_msgs
  
)
include_directories(
		include
		SYSTEM # everything after system is seen as system include (no compiler warnings)
		${catkin_INCLUDE_DIRS}
		${GAZEBO_INCLUDE_DIRS}
		${Boost_LIBRARY_DIRS})
add_library(lld src/lld.cpp)
add_dependencies(lld al5d_pckg_generate_messages_cpp)
add_executable(hld src/hld_main.cpp src/hld.cpp src/MatrixCalculations.cpp)
add_dependencies(hld al5d_pckg_generate_messages_cpp)

add_executable(controller src/Main.cpp src/Client.cpp src/MainApplication.cpp src/MatrixCalculations.cpp src/Shape.cpp src/Color.cpp src/Filter.cpp src/Programhandler.cpp src/Controller.cpp src/pose.cpp)
add_dependencies(controller al5d_pckg_generate_messages_cpp)
target_link_libraries(controller ${OpenCV_LIBS} -lpthread)



target_link_libraries(
  controller
  ${catkin_LIBRARIES}
)

target_link_libraries( 
  hld
  ${catkin_LIBRARIES} lld
)

add_dependencies(
  controller
  ${al5d_pckg_EXPORTED_TARGETS}
)


add_dependencies(
  hld
  ${al5d_pckg_EXPORTED_TARGETS}
)

