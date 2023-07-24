cmake_minimum_required(VERSION 3.23)

set(CMAKE_C_STANDARD 11)
project(SignalNet LANGUAGES C)
include_directories(Inc)

add_executable(SignalNet "test.c" "Src/graph.c" "Src/operators.c" "Src/signet_params.c")