cmake_minimum_required(VERSION 3.12)
project(clib2d)

set(CMAKE_CXX_STANDARD 14)

include_directories(final)
include_directories(c1p1)
include_directories(c1p2)
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
