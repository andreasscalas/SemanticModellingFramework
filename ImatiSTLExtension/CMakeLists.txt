cmake_minimum_required(VERSION 2.8.8)

project(ImatiSTLExtension)
set( IMATISTLEXLIB "${PROJECT_NAME}-lib")

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fpermissive -w")

set( IMATISTL ${IMATISTLEX}/ImatiSTL )
set( IMATISTL_H ${IMATISTL}/include/ImatiSTL )
set( KERNEL_H ${IMATISTL}/include/Kernel )
set( TMESH_H ${IMATISTL}/include/TMesh )
add_subdirectory(${IMATISTL})

set(Srcs ${IMATISTLEX}/src/extendedtrimesh)
set(Hdrs ${IMATISTLEX_H}/extendedtrimesh)

add_library(${IMATISTLEXLIB} ${Srcs} ${Hdrs})
set_target_properties(${IMATISTLEXLIB} PROPERTIES LINKER_LANGUAGE CXX)
target_include_directories(${IMATISTLEXLIB} PRIVATE
    ${IMATISTLEX_H}/
    ${IMATISTL_H}/
    ${KERNEL_H}/
    ${TMESH_H}/
    ${ANNOTATION_H}
    ${MAINFOLDER}/include/
    ${NANOFLANN}/include/
    ${NANOFLANN}/examples/
    ${SLICER}/include/
    ${EIGEN_H}
    ${SHAPEOP}/src
    ${DEFORMATION_H}/
    ${DATA_STRUCTURES_H}/
    ${ORTOOLS_H}/
)
target_link_libraries(${IMATISTLEXLIB} ImatiSTL-lib
    ${VTK_LIBRARIES})
