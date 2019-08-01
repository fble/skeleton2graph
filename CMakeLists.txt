cmake_minimum_required(VERSION 3.14)
project(skeleton2graph C)

set(CMAKE_C_STANDARD 11)

include_directories(. /home/stud-lifa1015/Projektarbeit/workspace/pace3d/lib /home/stud-lifa1015/Projektarbeit/pace3d/postprocessing)

add_executable(skeleton2graph
        skeleton2graph.c)

target_link_libraries(skeleton2graph m)
