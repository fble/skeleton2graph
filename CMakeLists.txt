cmake_minimum_required(VERSION 3.14)
project(skeleton2graph C)

set(CMAKE_C_STANDARD 11)

include_directories(. /home/stud-lifa1015/Projektarbeit/workspace/pace3d/lib /home/stud-lifa1015/Projektarbeit/pace3d/postprocessing)

add_executable(skeleton2graph
        skeleton2graph.c
        /home/stud-lifa1015/Projektarbeit/workspace/pace3d/lib/container/complexgraph.c
        /home/stud-lifa1015/Projektarbeit/workspace/pace3d/lib/container/hashmap.c
        /home/stud-lifa1015/Projektarbeit/workspace/pace3d/lib/container/node.c
        /home/stud-lifa1015/Projektarbeit/workspace/pace3d/lib/container/list.c
        /home/stud-lifa1015/Projektarbeit/workspace/pace3d/lib/container/vector3d.c
        /home/stud-lifa1015/Projektarbeit/workspace/pace3d/lib/wrapper.c
        /home/stud-lifa1015/Projektarbeit/workspace/pace3d/lib/exception.c)

target_link_libraries(skeleton2graph m -ldl)
