cmake_minimum_required(VERSION 3.0.2)
project(camera_node)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  sensor_msgs
  image_transport
  roscpp
  std_msgs
  message_generation
  geometry_msgs
  tf
  tf2
  tf2_ros
  assembly_robot_msgs
)

find_package(OpenCV REQUIRED)


add_service_files(FILES cam_Srv.srv)

generate_messages(
   DEPENDENCIES std_msgs )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES camera_node opencv tf assembly_robot_msgs
  CATKIN_DEPENDS cv_bridge roscpp std_msgs sensor_msgs image_transport
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  /opt/pylon/include
)
LINK_DIRECTORIES(/opt/pylon/lib)

add_executable(hole_find src/hole_find.cpp)
add_dependencies(hole_find ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(hole_find
   ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
 )

#add_executable(basler_camera src/basler_camera.cpp)
#add_dependencies(basler_camera ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(basler_camera
#   ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} pylonbase pylonutility GenApi_gcc_v3_1_Basler_pylon GCBase_gcc_v3_1_Basler_pylon
# )


