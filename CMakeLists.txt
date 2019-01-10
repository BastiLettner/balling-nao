cmake_minimum_required(VERSION 2.8.3)
project(balling-nao)


find_package(catkin REQUIRED cmake_modules COMPONENTS
  roscpp
  std_msgs
)

find_package(Boost REQUIRED COMPONENTS system thread)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

include_directories(
  ${catkin_INCLUDE_DIRS}
)

find_package(OpenCV REQUIRED)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} /usr/local/lib/cmake)
find_package(aruco REQUIRED)


add_library(NAO_MOTION src/cc/modules/motion.cc)
link_libraries(NAO_MOTION ${catkin_LIBRARIES})


add_library(NAO_VISTION src/cc/modules/vision.cc)
link_libraries(NAO_VISTION ${catkin_LIBRARIES})


add_library(NAO_SPEECH src/cc/modules/speech.cc)
link_libraries(NAO_SPEECH ${catkin_LIBRARIES})


file(GLOB STATE_FILES "*.cc")
add_library(NAO_CONTROLLER ${STATE_FILES} src/cc/modules/controller.cc)
link_libraries(NAO_STATES ${catkin_LIBRARIES} ${NAO_MOTION} ${NAO_VISION} ${NAO_SPEECH})


add_executable(BALLER src/main.cc)
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(BALLER ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${NAO_CONTROLLER})


