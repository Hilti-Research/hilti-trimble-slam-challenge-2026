cmake_minimum_required(VERSION 3.8)
project(challenge_tools_ros)

set(CMAKE_CXX_STANDARD 17)
if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)

# Stop build warnings related to missing IO components
set(PCL_FIND_QUIETLY ON)

# ## System dependencies are found with CMake's conventions
find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core)
find_package(PCL REQUIRED COMPONENTS common io filters kdtree surface visualization)
find_package(Eigen3 REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Add include directories
include_directories(
  include
  SYSTEM ${EIGEN3_INCLUDE_DIRS}
)


#############
## Install ##
#############

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/challenge_tools_ros/map_server.py
  scripts/challenge_tools_ros/map_server_demo.py
  scripts/challenge_tools_ros/apply_image_mask.py
  scripts/challenge_tools_ros/image_conversion_node.py
  scripts/challenge_tools_ros/rotate_image_180.py
  scripts/challenge_tools_ros/groundtruth_converter.py
  scripts/challenge_tools_ros/groundtruth_server.py
  scripts/challenge_tools_ros/challenge_tools_lib.py
  DESTINATION lib/${PROJECT_NAME}
)

#install(TARGETS 
#  pointcloud_to_pcd
#  DESTINATION lib/${PROJECT_NAME}
#)

## Mark cpp header files for installation
#install(DIRECTORY include/
#        DESTINATION include
#        FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h")

install(
  DIRECTORY config launch data floorplans groundtruth
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*procman.pmd"
                 PATTERN "*.yaml"
                 PATTERN "*.xml"
                 PATTERN "*.perspective"
                 PATTERN "*.rviz"
                 PATTERN "*.py"
                 PATTERN "*.png"
                 PATTERN "*.launch"
                 PATTERN "*.fbow"
                 PATTERN "*.csv"
                 PATTERN "*.bin"
)


#ament_export_include_directories(include)
#ament_export_libraries(
#  INSERT LIBRARY HERE
#)
ament_export_dependencies(Eigen3)
ament_package()
