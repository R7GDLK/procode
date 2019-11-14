cmake_minimum_required(VERSION 2.8.3)
project(search_object)

 add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs

)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES search_object
#  CATKIN_DEPENDS roscpp stdmsgs
#  DEPENDS system_lib
)

include_directories(
~/catkin_ws/include
${catkin_INCLUDE_DIRS}

)
## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/search_object.cpp
# )
LINK_DIRECTORIES(/home/robot/catkin_ws/lib/)

 add_executable(answer_class src/answer_class.cpp)
 add_dependencies(answer_class ${PORJECT_NAME}_generate_messages_cpp)
 target_link_libraries(answer_class 
  ${catkin_LIBRARIES} libmsc.so -ldl -pthread
  ${catkin_LIBRARIES} libasound.so -ldl -pthread
)
 add_executable(answer_obj src/answer_obj.cpp)
 add_dependencies(answer_obj ${PORJECT_NAME}_generate_messages_cpp)
 target_link_libraries(answer_obj 
  ${catkin_LIBRARIES} libmsc.so -ldl -pthread
  ${catkin_LIBRARIES} libasound.so -ldl -pthread
)
 add_executable(answer_class2 src/answer_class2.cpp)
 add_dependencies(answer_class2 ${PORJECT_NAME}_generate_messages_cpp)
 target_link_libraries(answer_class2 
  ${catkin_LIBRARIES} libmsc.so -ldl -pthread
  ${catkin_LIBRARIES} libasound.so -ldl -pthread
)
 add_executable(answer_obj2 src/answer_obj2.cpp)
 add_dependencies(answer_obj2 ${PORJECT_NAME}_generate_messages_cpp)
 target_link_libraries(answer_obj2 
  ${catkin_LIBRARIES} libmsc.so -ldl -pthread
  ${catkin_LIBRARIES} libasound.so -ldl -pthread
)

