cmake_minimum_required(VERSION 3.0.0)
project(whale VERSION 0.1.0)

include(CTest)
enable_testing()

add_executable(whale main.cpp)
target_compile_features(whale PRIVATE cxx_std_17)
set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
