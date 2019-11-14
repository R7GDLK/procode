cmake_minimum_required(VERSION 2.8.3)
project(photoshot)
add_compile_options(-std=c++11)
find_package( catkin REQUIRED )
find_package( OpenCV REQUIRED )
find_package( PCL REQUIRED )
find_package( catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
	genmsg
	actionlib_msgs
	actionlib
	sensor_msgs 
	message_filters 
	cv_bridge 
	image_transport 
	compressed_image_transport 
	compressed_depth_image_transport 
	kinect2_bridge
)
catkin_package( INCLUDE_DIRS include )
include_directories( include 
~/catkin_ws/include
${catkin_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRS}
${PCL_INCLUDE_DIRS}
${kinect2_bridge_INCLUDE_DIRS})
add_executable( photoshot src/photoshot.cpp )
add_dependencies( photoshot ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
target_link_libraries( photoshot
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  ${kinect2_bridge_LIBRARIES}
  ${catkin_LIBRARIES} -lcurl -ljsoncpp
  ${catkin_LIBRARIES} ~/catkin_ws/lib/libmsc.so -ldl -pthread
  ${catkin_LIBRARIES} ~/catkin_ws/lib/libasound.so -ldl -pthread
)
