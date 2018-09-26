cmake_minimum_required(VERSION 2.8.3)
project(lpvDS)

set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  mathlib
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)



## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES lpvDS
  CATKIN_DEPENDS roscpp mathlib
#  DEPENDS system_lib
)

###########
## Build ##
###########


set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(H_FILES     ${INCLUDE_DIR}/lpvDS.h
                ${INCLUDE_DIR}/utils.h
                ${INCLUDE_DIR}/GMM.h)

## Specify additional locations of header files
include_directories(include ${catkin_INCLUDE_DIRS})

## Declare a C++ library
set(SRCS src/lpvDS/lpvDS.cpp
         src/lpvDS/utils.cpp
         src/lpvDS/GMM.cpp)

## Declare a cpp library
add_library(${PROJECT_NAME} ${H_FILES}  ${SRCS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

## Declare a C++ executable
add_executable(test_lpvDS      src/test_lpvDS.cpp)
add_executable(test_lpvDS_node src/test_lpvDS_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(test_lpvDS      lpvDS ${catkin_LIBRARIES})
target_link_libraries(test_lpvDS_node lpvDS ${catkin_LIBRARIES})
