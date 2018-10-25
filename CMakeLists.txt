cmake_minimum_required(VERSION 3.9)
project(FinalProject)

set(CMAKE_CXX_STANDARD 17)

add_executable(FinalProject main.cpp Point_t.h 3DModeling.h Vector_t.h 3DModeling.cpp Point_t.cpp Vector_t.cpp Line_t.cpp Line_t.h Plane_t.cpp Plane_t.h Building_t.cpp Building_t.h SystemParameters.h UtilityMethods.cpp UtilityMethods.h Path_t.cpp Path_t.h PhyLink.cpp PhyLink.h)