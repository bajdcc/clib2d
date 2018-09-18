cmake_minimum_required(VERSION 3.12)
project(clib2d)

set(CMAKE_CXX_STANDARD 14)

link_libraries(freeglut opengl32 glu32)

add_executable(clib2d-final
        final/cbody.cpp
        final/cbody.h
        final/cjoint.cpp
        final/cjoint.h
        final/cpair.cpp
        final/cpair.h
        final/ctypes.cpp
        final/ctypes.h
        final/cworld.cpp
        final/cworld.h
        final/main.cpp)

add_executable(clib2d-c1p1
        c1p1/main.cpp)

add_executable(clib2d-c1p2
        c1p2/main.cpp)

add_executable(clib2d-c1p3
        c1p3/main.cpp)

add_executable(clib2d-c2p1
        c2p1/main.cpp)

add_executable(clib2d-c2p2
        c2p2/main.cpp)

add_executable(clib2d-c2p3
        c2p3/main.cpp)

add_executable(clib2d-c3p1
        c3p1/main.cpp)

add_executable(clib2d-c4p1
        c4p1/main.cpp)
