cmake_minimum_required(VERSION 3.27)
project(Projeto_2)

set(CMAKE_CXX_STANDARD 17)

include_directories(.)

add_executable(Projeto_2
        src/Graph.cpp
        src/Graph.h
        src/parse.h
        src/parse.cpp
        src/main.cpp
        src/menu.h
        src/Actions.cpp
        src/Actions.h
        tests/testTSPBacktracking.cpp
        src/Utils.h
        src/Utils.cpp
)
