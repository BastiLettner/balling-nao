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
catkin_package(

)
find_package(OpenCV REQUIRED)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} /usr/local/lib/cmake)
find_package(aruco REQUIRED)


add_library(MOTION src/cc/modules/motion.cc)
link_libraries(MOTION ${catkin_LIBRARIES})


add_library(VISTION src/cc/modules/vision.cc)
link_libraries(VISTION ${catkin_LIBRARIES})


add_library(SPEECH src/cc/modules/speech.cc)
link_libraries(SPEECH ${catkin_LIBRARIES})


add_library(CONTROLLER
        src/cc/modules/controller.cc
        src/cc/states/search_defender.cc
        src/cc/states/take_ball.cc
        src/cc/states/state.cc
        src/cc/states/request_task.cc
        src/cc/states/request_ball.cc
        src/cc/states/move_into_dunk_pos.cc
        src/cc/states/move_into_throw_dist.cc
        src/cc/states/execute_dunk_motion.cc
        src/cc/states/execute_throw_motion.cc
        src/cc/states/done.h
        src/cc/states/detect_hoop.cc
        src/cc/states/avoid_defener_routine.cc)
link_libraries(CONTROLLER ${catkin_LIBRARIES} ${MOTION} ${VISION} ${SPEECH})


add_executable(BALLER src/main.cc)
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(BALLER ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${CONTROLLER})


