cmake_minimum_required(VERSION 2.8.3)
project(indoor_navigation)
add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
	genmsg
	actionlib_msgs
	actionlib
	rostime 
	sensor_msgs 
	message_filters 
	cv_bridge 
	image_transport 
	compressed_image_transport 
	compressed_depth_image_transport 
	kinect2_bridge
	tf
)

find_package(OpenCV REQUIRED)
find_package(OpenMP)
find_package(PCL REQUIRED)

catkin_package()

if(OPENMP_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

include_directories(
~/catkin_ws/include
${catkin_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRS}
${PCL_INCLUDE_DIRS}
${kinect2_bridge_INCLUDE_DIRS}
)
LINK_DIRECTORIES(/home/robot/catkin_ws/lib/)
add_executable(indoor_navigation src/indoor_navigation.cpp)
add_dependencies(indoor_navigation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(indoor_navigation
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  ${kinect2_bridge_LIBRARIES}
  ${catkin_LIBRARIES} -lcurl -ljsoncpp
  ${catkin_LIBRARIES} libmsc.so -ldl -pthread
  ${catkin_LIBRARIES} libasound.so -ldl -pthread

)
