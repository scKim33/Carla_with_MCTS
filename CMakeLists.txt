cmake_minimum_required (VERSION 3.0.2)

# Set flags
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Message out the current compiler flags
message("CMAKE_CXX_FLAGS_DEBUG is ${CMAKE_CXX_FLAGS_DEBUG}")
message("CMAKE_CXX_FLAGS_RELEASE is ${CMAKE_CXX_FLAGS_RELEASE}")

project(vanilla_mcts)

find_package(catkin REQUIRED COMPONENTS
             roscpp
             std_msgs
             cmake_modules
            )
find_package(Eigen REQUIRED)
find_package(Boost REQUIRED COMPONENTS)

catkin_package(
  CATKIN_DEPENDS 
    roscpp 
    std_msgs
    geometry_msgs
  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_executable(vanilla_mcts src/mcts_random.cpp src/main.cpp src/rviz_utils.cpp)

target_link_libraries(vanilla_mcts
                      ${catkin_LIBRARIES}
                      ${Boost_LIBRARIES}
                      ${Eigen_LIBRARIES}
                      )
