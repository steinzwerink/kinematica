cmake_minimum_required(VERSION 3.7.2)

project(vision)
find_package(OpenCV 3.2.0 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
roscpp
actionlib
std_msgs
message_generation
message_runtime
al5d_pckg
actionlib_msgs)
find_package(Boost REQUIRED COMPONENTS system)

#add_action_files(
#  DIRECTORY action
#  FILES communicator.action
#)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs
  std_msgs
message_generation
message_runtime
al5d_pckg
)

include_directories(
		include
		SYSTEM # everything after system is seen as system include (no compiler warnings)
		${catkin_INCLUDE_DIRS}
		${GAZEBO_INCLUDE_DIRS}
		${Boost_LIBRARY_DIRS})

add_executable(vision 
src/Main.cpp 
)

add_dependencies(${PROJECT_NAME} vision_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
		${catkin_LIBRARIES}
		${GAZEBO_LIBRARIES}
		${Boost_LIBRARIES})
target_link_libraries(vision ${OpenCV_LIBS} -lpthread)
set(CMAKE_CXX_FLAGS "-std=c++14 -Wall -Werror -Wextra -Wconversion")

target_link_libraries(
  vision
  ${catkin_LIBRARIES}
)

add_dependencies(
  vision
  ${vision_EXPORTED_TARGETS}
)



