cmake_minimum_required(VERSION 3.0.0)
project(whale VERSION 0.1.0)
set(CMAKE_CXX_STANDARD 17)
include(CTest)
enable_testing()

find_package(LLVM 14 REQUIRED CONFIG)

message(STATUS "Found LLVM ${LLVM_PACKAGE_VERSION}")
message(STATUS "Using LLVMConfig.cmake in: ${LLVM_DIR}")

include_directories(${LLVM_INCLUDE_DIRS})
separate_arguments(LLVM_DEFINITIONS_LIST NATIVE_COMMAND ${LLVM_DEFINITIONS})
add_definitions(${LLVM_DEFINITIONS_LIST})

add_executable(whale main.cpp compile.cpp parser.cpp print.cpp runner.cpp)

execute_process(COMMAND llvm-config-14 --system-libs --link-shared --libs all
                OUTPUT_VARIABLE llvm_libs
                OUTPUT_STRIP_TRAILING_WHITESPACE)

                message(STATUS "LLVM linked with ${llvm_libs}")

target_link_libraries(whale ${llvm_libs} ${targets})

target_compile_options(whale PUBLIC)
set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)

file(COPY tests DESTINATION .)