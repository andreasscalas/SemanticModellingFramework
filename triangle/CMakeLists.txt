cmake_minimum_required(VERSION 2.8.8)

project(Triangle)
set( Srcs
    ${TRIANGLE}/triangle.cpp
    ${TRIANGLE}/tricall.c
)
set( Hdrs
    ${TRIANGLE}/triangle.h
)

add_definitions(-DTRILIBRARY -DANSI_DECLARATORS -DVOID=void -DREAL=double -DNO_TIMER=true)
add_library(${PROJECT_NAME}-lib ${Srcs} ${Hdrs})
target_include_directories(${PROJECT_NAME}-lib PRIVATE ${TRIANGLE})
