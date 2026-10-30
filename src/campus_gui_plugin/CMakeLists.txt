cmake_minimum_required(VERSION 3.10)
project(campus_gui_plugin)

find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)


include_directories(${GAZEBO_INCLUDE_DIRS} ${Qt5Core_INCLUDE_DIRS} ${Qt5Widgets_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
add_definitions(${GAZEBO_CXX_FLAGS})

add_library(campus_labels SHARED src/campus_labels.cpp)
target_link_libraries(campus_labels Qt5::Core Qt5::Widgets ${GAZEBO_LIBRARIES})

# libcampus_labels.so ends up in install/campus_gui_plugin/lib/
install(TARGETS campus_labels LIBRARY DESTINATION lib)

ament_package()
