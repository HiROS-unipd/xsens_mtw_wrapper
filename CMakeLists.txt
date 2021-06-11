cmake_minimum_required(VERSION 3.0)
project(hiros_xsens_mtw_wrapper)

# Compile as C++14
add_compile_options(-std=c++14)

find_package(cmake_modules REQUIRED)

list(INSERT CMAKE_MODULE_PATH
            0
            "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
find_package(Xsens REQUIRED)
# Find catkin macros and libraries if COMPONENTS list like find_package(catkin
# REQUIRED COMPONENTS xyz) is used, also find other catkin packages
find_package(catkin REQUIRED
             COMPONENTS geometry_msgs
                        message_generation
                        roscpp
                        sensor_msgs
                        std_msgs
                        tf2
                        tf2_ros)

# System dependencies are found with CMake's conventions

# ##############################################################################
# Declare ROS messages, services and actions ##
# ##############################################################################

# Generate messages in the 'msg' folder
add_message_files(FILES Euler.msg
                        MIMU.msg
                        MIMUArray.msg)

# Generate services in the 'srv' folder
add_service_files(FILES ResetOrientation.srv
                        StartRecording.srv
                        StopRecording.srv)

# Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
                  geometry_msgs
                  sensor_msgs
                  std_msgs)

# ##############################################################################
# catkin specific configuration ##
# ##############################################################################
# The catkin_package macro generates cmake config files for your package Declare
# things to be passed to dependent projects INCLUDE_DIRS: uncomment this if your
# package contains header files LIBRARIES: libraries you create in this project
# that dependent projects also need CATKIN_DEPENDS: catkin_packages dependent
# projects also need DEPENDS: system dependencies of this project that dependent
# projects also need
catkin_package(CATKIN_DEPENDS
               geometry_msgs
               message_runtime
               roscpp
               sensor_msgs
               std_msgs
               tf2
               tf2_ros)

# ##############################################################################
# Build ##
# ##############################################################################
# Display additional files in qtcreator
file(GLOB_RECURSE
     FILES_LIST
     "*.launch"
     "*.msg"
     "*.srv"
     "*.xml")
add_custom_target(additional_files_${PROJECT_NAME} SOURCES ${FILES_LIST})

# Declare a C++ executable With catkin_make all packages are built within a
# single CMake context The recommended prefix ensures that target names across
# packages don't collide add_executable(${PROJECT_NAME}_node
add_executable(${PROJECT_NAME}_node
               src/MtwCallback.cpp
               src/Synchronizer.cpp
               src/WirelessMasterCallback.cpp
               src/Wrapper.cpp
               src/main.cpp
               src/utils.cpp)

# Rename C++ executable without prefix The above recommended prefix causes long
# target names, the following renames the target back to the shorter version for
# ease of user use e.g. "rosrun someones_pkg node" instead of "rosrun
# someones_pkg someones_pkg_node"
set_target_properties(${PROJECT_NAME}_node
                      PROPERTIES OUTPUT_NAME
                                 ${PROJECT_NAME}
                                 PREFIX
                                 "")

# Add cmake target dependencies of the executable same as for the library above
add_dependencies(${PROJECT_NAME}_node
                 ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})

# Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
                      PUBLIC ${catkin_LIBRARIES}
                      PRIVATE ${Xsens_LIBRARIES})

set_target_properties(${PROJECT_NAME}_node
                      PROPERTIES PUBLIC_HEADER include/xsens_mtw/Wrapper.h)

target_include_directories(${PROJECT_NAME}_node
                           PRIVATE include
                                   ${Xsens_INCLUDE_DIRS}
                                   ${catkin_INCLUDE_DIRS})

# ##############################################################################
# Install ##
# ##############################################################################
install(TARGETS ${PROJECT_NAME}_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES include/xsens_mtw/Wrapper.h
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY config launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
