cmake_minimum_required(VERSION 2.8.8)

project(andreaslib)

find_package(VTK COMPONENTS
    vtkCommonCore
    vtkFiltersCore
    vtkInfovisCore
    vtkInteractionStyle
    vtkViewsQt
)

include(${VTK_USE_FILE})

if("${VTK_QT_VERSION}" STREQUAL "")
  message(FATAL_ERROR "VTK was not built with Qt")
endif()

set( MAINFOLDER ${CMAKE_SOURCE_DIR} )
SET( CONSTRAINTS ${MAINFOLDER}/Constraints )
set( ANNOTATION ${MAINFOLDER}/Annotation )
set( ANNOTATION_H ${MAINFOLDER}/Annotation/include/ )
set( DEFORMATION ${MAINFOLDER}/Deformation )
set( DEFORMATION_H ${MAINFOLDER}/Deformation/include )
set( IMATISTLEX ${MAINFOLDER}/ImatiSTLExtension )
set( IMATISTLEX_H ${MAINFOLDER}/ImatiSTLExtension/include )
set( IMATISTL_H ${MAINFOLDER}/ImatiSTLExtension/ImatiSTL/include/ )
set( SLICER ${MAINFOLDER}/Slicer/)
set( SLICER_H ${MAINFOLDER}/Slicer/include/)
set( NANOFLANN ${MAINFOLDER}/nanoflann/)
set( TRIANGLE ${MAINFOLDER}/triangle/ )
set( SHAPEOP ${MAINFOLDER}/ShapeOp.0.1.0/libShapeOp/)

set( DATA_STRUCTURES ${MAINFOLDER}/DataStructures )
set( DATA_STRUCTURES_H ${MAINFOLDER}/DataStructures/include )

add_definitions(-DTRIANGLE_HEADER="${TRIANGLE}/triangle.h")

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fopenmp")
find_library(MATHGEO_LIB NAMES MATHGEOLIB MathGeoLib mathgeolib MATHGEO MathGeo mathGeo mathgeo)
find_path(MATHGEOLIB_H MathGeoLib-master/src)
find_package(RapidJSON)
find_path(EIGEN_H Eigen)
find_path(RAPIDJSON_H rapidjson-master/include/)
find_library(TINYXML2_LIB NAMES tinyXML2 tinyxml2 TINYXML2 tinyXML_2 tinyxml_2 TINYXML_2)
find_path(TINYXML2_H tinyxml2-master/)

add_subdirectory(${IMATISTLEX})
add_subdirectory(${ANNOTATION})
add_subdirectory(${DEFORMATION})
add_subdirectory(${SLICER})
add_subdirectory(${SHAPEOP})
add_subdirectory(${TRIANGLE})
add_subdirectory(${DATA_STRUCTURES})


set( NONRIGID_REGISTRATION ${MAINFOLDER}/NonRigidRegistration/ )
add_subdirectory(${NONRIGID_REGISTRATION})


# Set your files and resources here
file(GLOB Srcs "${MAINFOLDER}/src/*.c*" "${MAINFOLDER}/src/Styles/*.c*")
file(GLOB Hdrs "${MAINFOLDER}/include/*.h" "${MAINFOLDER}/include/Styles/*.h")
file(GLOB UIs "${MAINFOLDER}/gui/*.ui")

set( Docs ${CMAKE_CURRENT_SOURCE_DIR}/../docs/Doxyfile.in)

set( QRCs ${CMAKE_SOURCE_DIR}/Icons/icons.qrc )


# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)
find_package(Qt5 COMPONENTS Widgets REQUIRED QUIET)
qt5_wrap_ui(UI_Srcs ${UIs})
qt5_add_resources(QRC_Srcs ${QRCs} )

source_group("Resources" FILES
${UIs}
${QRCs}
${EXE_ICON} # Not present
)

source_group("Generated" FILES
${UI_Srcs}
${MOC_Srcs}
${QRC_Srcs}
)

# CMAKE_AUTOMOC in ON so the MocHdrs will be automatically wrapped.
add_executable(${PROJECT_NAME} ${Srcs} ${Hdrs} ${UI_Srcs} ${MOC_Hdrs} ${QRC_Srcs} )
qt5_use_modules(${PROJECT_NAME} Core Gui Widgets)
target_link_libraries(${PROJECT_NAME} #ConstraintsLib-lib
    Annotation-lib
    ImatiSTLExtension-lib
    Slicer-lib
    Deformation-lib
    DataStructures-lib
    Triangle-lib
    ShapeOp
    graphene_pointset_processing
    graphene_surface_mesh
    graphene_surface_mesh_algorithms
    graphene_surface_mesh_eigen_algorithms
    graphene_surface_mesh_scene_graph
    ${VTK_LIBRARIES} ${MATHGEO_LIB} ${TINYXML2_LIB}
)
target_include_directories(${PROJECT_NAME} PRIVATE
  ${CMAKE_CURRENT_BINARY_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${RAPIDJSON_H}
  ${TINYXML2_H}
  ${MATHGEOLIB_H}
  ${EIGEN_H}
  ${SHAPEOP_H}
  ${CMAKE_SOURCE_DIR}/..
  ${MAINFOLDER}/include/
  ${MAINFOLDER}/include/Styles
  ${SLICER_H}/
  ${ANNOTATION_H}
  ${DEFORMATION_H}
  ${DATA_STRUCTURES_H}
  ${IMATISTLEX_H}/
  ${IMATISTL_H}/ImatiSTL/
  ${IMATISTL_H}/Kernel/
  ${IMATISTL_H}/TMesh/
  ${NANOFLANN}/include/
  ${NANOFLANN}/examples/
  ${SHAPEOP}/src
  ${NONRIGID_REGISTRATION}/
)
