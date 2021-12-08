cmake_minimum_required(VERSION 3.10)

project(Localization)
set(source_code src/NDT.cpp)
add_executable(localization ${source_code})
#target_link_libraries(localization xx)