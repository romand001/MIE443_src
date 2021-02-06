cmake_minimum_required(VERSION 2.8)
project(mie443_contest1)

add_compile_options(-std=c++11)

#set(OpenCV_DIR "/usr/share/OpenCV")

find_package(OpenCV 3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
	cv_bridge
	genmsg
	image_transport
	sensor_msgs
    kobuki_msgs
    tf)

generate_messages(DEPENDENCIES sensor_msgs kobuki_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES main_class_node
  CATKIN_DEPENDS ${PKG_DEPS}
)

include_directories(include ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})


add_library(main_class_node src/main_node_class.cpp)
add_library(map_class src/map_class.cpp)


add_executable(main_node_instance src/main_node_instance.cpp)
target_link_libraries(main_node_instance ${catkin_LIBRARIES} ${OpenCV_LIB} main_class_node map_class)
