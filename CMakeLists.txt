cmake_minimum_required(VERSION 2.8.3)
project(dylan_msc)

add_compile_options(-std=c++11)

# FOLLOWING LINE ADDED FOR USE WITH TURTLEBOT
# set(PCL_DIR "/usr/lib/aarch64-linux-gnu/cmake/pcl")

# FOLLOWING ADDED FOR USE WITH MSI LAPTOP
# set(PCL_DIR "/usr/lib/x86_64-linux-gnu/cmake/pcl")

find_path(PCL_DIR
    PCLConfig.cmake pcl-config.cmake
    /usr/lib/aarch64-linux-gnu/cmake/pcl /usr/lib/x86_64-linux-gnu/cmake/pcl
)

set(PCL_DIR )

find_package(catkin REQUIRED COMPONENTS 
    roscpp 
    rospy 
    std_msgs
    geometry_msgs
    message_generation
    PCL
)

add_message_files(
    FILES
    obj.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
)

catkin_package(
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs message_runtime
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)


add_executable(segment_cloud src/segment_cloud.cpp)
add_executable(distance_crop src/distance_crop.cpp)
target_link_libraries (distance_crop ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries (segment_cloud ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(segment_cloud dylan_msc_generate_messages_cpp ${${PROJECT_NAME}_EXPORTED_TARGETS}  ${catkin_EXPORTED_TARGETS})
add_dependencies(distance_crop dylan_msc_generate_messages_cpp ${${PROJECT_NAME}_EXPORTED_TARGETS}  ${catkin_EXPORTED_TARGETS})

