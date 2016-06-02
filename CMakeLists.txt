cmake_minimum_required(VERSION 2.8.3)
project(pbvs_arm_servo)

# Tell CMake where to find "FindNAOqi.cmake"
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
  visp_bridge
)

find_package(VISP REQUIRED)

find_package(NAOqi QUIET COMPONENTS
    alcommon
    alvision
    alproxies
    alerror
    alvalue
    qi
    qitype
)


# Make sure Boost.Filesystem v3 is used.
add_definitions(-DBOOST_FILESYSTEM_VERSION=3)

# Add ViSP definitions.
add_definitions(${VISP_DEFINITIONS})


find_package(visp_naoqi REQUIRED )
find_package(ROMEO_TK REQUIRED)



catkin_package(
  
#  INCLUDE_DIRS
#  include
  
  LIBRARIES
  pbvs_arm_servo
  
  CATKIN_DEPENDS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf

  DEPENDS 
  VISP
  visp_naoqi
  romeo_tk
  visp_bridge
)

include_directories(include)


if( NAOqi_FOUND)
    message(STATUS "VISPNAOQIBRIDGE: NAOqi C++ SDK found; C++ nodes will be built")
   
else()
    message(STATUS "VISPNAOQIBRIDGE:Cannot find NAOqi C++ sdk; C++ nodes will NOT be built")
endif()

include_directories(SYSTEM
  ${VISP_INCLUDE_DIRS}
  ${VISP_NAOQI_INCLUDE_DIRS}
  ${NAOqi_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${ROMEO_TK_INCLUDE_DIRS}
)


## Make sure Boost.Filesystem v3 is used.
add_definitions(-DBOOST_FILESYSTEM_VERSION=3)

# Add package definitions
add_definitions(${VISP_DEFINITIONS})

# Declare a cpp library
add_library(pbvs_arm_servo
  include/pbvs_arm_servo.h
  src/pbvs_arm_servo.cpp
)



## Specify libraries to link a library or executable target against
#target_link_libraries(pbvs_arm_servo_node
#  ${catkin_LIBRARIES}
#  ${VISP_LIBRARIES}
#  ${NAOqi_LIBRARIES}
#  ${VISP_NAOQI_LIBRARIES}
##  pbvs_arm
#)
target_link_libraries(pbvs_arm_servo
  ${catkin_LIBRARIES}
  ${VISP_LIBRARIES}
  ${NAOqi_LIBRARIES}
  ${VISP_NAOQI_LIBRARIES}
  ${ROMEO_TK_LIBRARIES}
)

## Declare a cpp executable
#add_executable(pbvs_arm_servo_node src/pbvs_arm_servo.cpp)
add_executable(pbvs_arm_main_node src/main.cpp)


  target_link_libraries(pbvs_arm_main_node
    ${resource_retriever_LIBRARIES}
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    pbvs_arm_servo
    )



#message("NAOqi VISPNAOQIBRIDGE: Hint ${NAOqi_LIBRARY_HINTS}")
#message("NAOqi VISPNAOQIBRIDGE: LIB ${NAOqi_LIBRARY}")
