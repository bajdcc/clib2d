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

add_executable(clib2d-c4p2
        c4p2/main.cpp)

add_executable(clib2d-c5p1
        c5p1/main.cpp
        c5p1/memory.h
        c5p1/memory_gc.h
        c5p1/types.h
        c5p1/types.cpp
        c5p1/clexer.h
        c5p1/clexer.cpp
        c5p1/cparser.h
        c5p1/cparser.cpp
        c5p1/cast.h
        c5p1/cast.cpp
        c5p1/cvm.cpp
        c5p1/cvm.h
        c5p1/csub.cpp
        c5p1/csub.h
        c5p1/v2.cpp
        c5p1/v2.h
        c5p1/c2d.cpp
        c5p1/c2d.h
        c5p1/m2.cpp
        c5p1/m2.h
        c5p1/c2dbody.cpp
        c5p1/c2dbody.h
        c5p1/c2dpolygon.cpp
        c5p1/c2dpolygon.h
        c5p1/c2dcircle.cpp
        c5p1/c2dcircle.h
        c5p1/c2djoint.cpp
        c5p1/c2djoint.h
        c5p1/c2drevolute.cpp
        c5p1/c2drevolute.h
        c5p1/c2dworld.cpp
        c5p1/c2dworld.h
        c5p1/c2dcontact.cpp
        c5p1/c2dcontact.h
        c5p1/c2dcollision.cpp
        c5p1/c2dcollision.h)

add_executable(clib2d-c5p1-test
        c5p1/test.cpp
        c5p1/memory.h
        c5p1/memory_gc.h
        c5p1/types.h
        c5p1/types.cpp
        c5p1/clexer.h
        c5p1/clexer.cpp
        c5p1/cparser.h
        c5p1/cparser.cpp
        c5p1/cast.h
        c5p1/cast.cpp
        c5p1/cvm.cpp
        c5p1/cvm.h
        c5p1/csub.cpp
        c5p1/csub.h)
