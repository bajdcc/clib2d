cmake_minimum_required(VERSION 3.12)
project(CGFluid)

set(CMAKE_CXX_STANDARD 14)

add_executable(CGFluid main.cpp cworld.cpp cworld.h cbody.cpp cbody.h cjoint.cpp cjoint.h cpair.cpp cpair.h ctypes.cpp ctypes.h)
target_link_libraries(CGFluid freeglut opengl32 glu32)