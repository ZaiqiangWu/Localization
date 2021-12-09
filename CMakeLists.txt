cmake_minimum_required(VERSION 3.5)

project(Localization)
find_package( PCL 1.8 REQUIRED )
include_directories( ${PCL_INCLUDE_DIRS} )
set(CMAKE_CXX_STANDARD 11)
set(source_code src/NDT.cpp)
add_executable(localization ${source_code})
target_link_libraries(localization ${PCL_LIBRARIES})