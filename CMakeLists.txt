cmake_minimum_required(VERSION 3.1 FATAL_ERROR)
project(bug_navigation)

set(CMAKE_BUILD_TYPE RelWithDebInfo)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

find_package(mbot_bridge REQUIRED)

# Robot hits the spot executable.
add_executable(hit_the_spot src/hit_the_spot.cpp
  src/common/utils.cpp
)
target_link_libraries(hit_the_spot
  mbot_bridge_cpp
)
target_include_directories(hit_the_spot PRIVATE
 include
)

# Bug navigation executable.
add_executable(bug_navigation src/bug_navigation.cpp
  src/common/utils.cpp
)
target_link_libraries(bug_navigation
  mbot_bridge_cpp
)
target_include_directories(bug_navigation PRIVATE
  include
)

if (CMAKE_BUILD_TYPE MATCHES Test)
endif()