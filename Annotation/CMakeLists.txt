cmake_minimum_required(VERSION 2.8.8)
project(Annotation)
set( ANNOTATIONLIB "${PROJECT_NAME}-lib")
set( ANNOTATION_SRC ${MAINFOLDER}/Annotation/src/)
set( ANNOTATION_H ${MAINFOLDER}/Annotation/include/)
set( NANOFLANN ${MAINFOLDER}/nanoflann/)

file(GLOB Srcs "${ANNOTATION_SRC}/*.c*")
file(GLOB Hdrs "${ANNOTATION_H}/*.h")
include_directories(
    ${ANNOTATION_H}/
    ${MAINFOLDER}/ImatiSTLExtension/include/
    ${IMATISTL_H}/ImatiSTL/
    ${IMATISTL_H}/Kernel/
    ${IMATISTL_H}/TMesh/
    ${DATA_STRUCTURES_H}/
    ${SLICER_H}/
    ${DEFORMATION_H}/
    ${MAINFOLDER}/include/
    ${NANOFLANN}/include/
    ${NANOFLANN}/examples/
    ${ORTOOLS_H}/
)
add_library(${ANNOTATIONLIB} ${Srcs} ${Hdrs})
target_include_directories(${ANNOTATIONLIB} PRIVATE
    ${ANNOTATION_H}/
    ${MAINFOLDER}/ImatiSTLExtension/include/
    ${IMATISTL_H}/ImatiSTL/
    ${IMATISTL_H}/Kernel/
    ${IMATISTL_H}/TMesh/
    ${SLICER_H}
    ${MAINFOLDER}/include/
    ${NANOFLANN}/include/
    ${NANOFLANN}/examples/
    ${EIGEN_H}
    ${SHAPEOP}/src
    ${ORTOOLS_H}/
)

target_link_libraries(${ANNOTATIONLIB} ImatiSTLExtension-lib DataStructures-lib)
