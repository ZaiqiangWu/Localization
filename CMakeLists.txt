cmake_minimum_required(VERSION 3.5)

project(Localization)
set(CMAKE_CXX_STANDARD 11)
set(source_code src/NDT.cpp)
add_executable(localization ${source_code})
#target_link_libraries(localization xx)