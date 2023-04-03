cmake_minimum_required(VERSION 2.8.8)

project(Slicer)
set( SLICERLIB "${PROJECT_NAME}-lib")

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fpermissive -w")

set( SLICER_SRC ${SLICER}/src/ )
set( SLICER_H ${SLICER}/include/ )

set(Srcs
    ${SLICER_SRC}/clusterthread
    ${SLICER_SRC}/contourcontainmenttreenode
    ${SLICER_SRC}/maarcpath
    ${SLICER_SRC}/manode
    ${SLICER_SRC}/slice
    ${SLICER_SRC}/slicer
    ${SLICER_SRC}/slicingthread
    ${SLICER_SRC}/trianglehelper
)
set(Hdrs
    ${SLICER_H}/clusterthread
    ${SLICER_H}/contourcontainmenttreenode
    ${SLICER_H}/maarcpath
    ${SLICER_H}/manode
    ${SLICER_H}/multiplyconnectednode
    ${SLICER_H}/slice
    ${SLICER_H}/slicer
    ${SLICER_H}/slicingthread
    ${SLICER_H}/treenode
    ${SLICER_H}/trianglehelper
)


add_library(${SLICERLIB} ${Srcs} ${Hdrs})
set_target_properties(${SLICERLIB} PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(${SLICERLIB} ImatiSTLExtension-lib Triangle-lib)

target_include_directories(${SLICERLIB} PRIVATE
    ${MAINFOLDER}/include/
    ${IMATISTLEX_H}/
    ${IMATISTL_H}/ImatiSTL/
    ${IMATISTL_H}/Kernel/
    ${IMATISTL_H}/TMesh/
    ${ANNOTATION_H}/
    ${NANOFLANN}/include/
    ${NANOFLANN}/examples/
    ${SLICER_H}/
    ${EIGEN_H}
    ${SHAPEOP}/src
    ${DEFORMATION_H}/
    ${ORTOOLS_H}/
)
