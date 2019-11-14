cmake_minimum_required(VERSION 2.8.3)
project(spr_voice)

add_compile_options(-std=c++11)
find_package(catkin REQUIRED)

# find_package(Boost REQUIRED COMPONENTS system)
# catkin_python_setup()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  genmsg
  actionlib_msgs
  actionlib
)

find_package(catkin REQUIRED COMPONENTS 
rostime 
sensor_msgs 
message_filters 
cv_bridge 
image_transport 
compressed_image_transport 
compressed_depth_image_transport 
kinect2_bridge
)

## System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED)
find_package(OpenMP)
find_package(PCL REQUIRED)

if(OPENMP_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES spr_voice
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include 
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${kinect2_bridge_INCLUDE_DIRS}
)

add_executable(spr_voice src/spr_voice.cpp)

execute_process(COMMAND find ${PROJECT_SOURCE_DIR} -type f OUTPUT_VARIABLE FILES_STRING)
string(REPLACE "\n" ";" FILES_LIST ${FILES_STRING})
add_custom_target(additional_files_${PROJECT_NAME}
  SOURCES
  ${FILES_LIST}
)

add_dependencies(spr_voice ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(spr_voice
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  ${kinect2_bridge_LIBRARIES}
  ${catkin_LIBRARIES} -lcurl -ljsoncpp
  ${catkin_LIBRARIES} /home/robot/catkin_ws/lib/libmsc.so -ldl -pthread
  ${catkin_LIBRARIES} /home/robot/catkin_ws/lib/libasound.so -ldl -pthread
  ${catkin_LIBRARIES} /home/robot/catkin_ws/lib/libasound.so.2 -ldl -pthread
  ${catkin_LIBRARIES} /home/robot/catkin_ws/lib/libasound.so.2.0.0 -ldl -pthread



#  ${catkin_LIBRARIES} /home/robot/catkin_ws/lib/libholiday.so -ldl -pthread
#  ${catkin_LIBRARIES} /home/robot/catkin_ws/lib/libSeetaFaceCropper200.so -ldl -pthread
#  ${catkin_LIBRARIES} /home/robot/catkin_ws/lib/libSeetaspr_voiceector200.so -ldl -pthread
#  ${catkin_LIBRARIES} /home/robot/catkin_ws/lib/libSeetaFaceRecognizer200.so -ldl -pthread
#  ${catkin_LIBRARIES} /home/robot/catkin_ws/lib/libSeetaPointDetector200.so -ldl -pthread

)
## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/spr_voice.cpp
# )

# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_executable(${PROJECT_NAME}_node src/spr_voice_node.cpp)

# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

install(TARGETS spr_voice
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############
# catkin_add_gtest(${PROJECT_NAME}-test test/test_spr_voice.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

