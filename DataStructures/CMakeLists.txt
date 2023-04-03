cmake_minimum_required(VERSION 2.8.8)
project(DataStructures CXX)
set( DATA_STRUCTURESLIB "${PROJECT_NAME}-lib")
set( DATA_STRUCTURES_SRC ${MAINFOLDER}/DataStructures/src/)
#set( Srcs
#    ${DATA_STRUCTURES_SRC}/list.cpp)

set( GraphMainSrcs
    ${DATA_STRUCTURES_SRC}/testGraph.cpp)

set( TreeMainSrcs
    ${DATA_STRUCTURES_SRC}/testTree.cpp)

file(GLOB Hdrs "${DATA_STRUCTURES_H}/*.h")

include_directories(
    ${DATA_STRUCTURES_H}
    ${ANNOTATION_H}
)

add_library(${DATA_STRUCTURESLIB} ${Srcs} ${Hdrs})
SET_TARGET_PROPERTIES(${DATA_STRUCTURESLIB} PROPERTIES LINKER_LANGUAGE CXX)
add_executable(testGraph ${GraphMainSrcs} ${Srcs} ${Hdrs})
add_executable(testTree ${TreeMainSrcs} ${Srcs} ${Hdrs})
target_include_directories(${DATA_STRUCTURESLIB} PRIVATE ${DATA_STRUCTURES_H} ${ANNOTATION_H})
