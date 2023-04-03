cmake_minimum_required(VERSION 2.8.8)

project(andreaslib)

find_package(VTK REQUIRED COMPONENTS
    CommonCore
    FiltersCore
    FiltersGeneral
    FiltersGeometry
    FiltersModeling
    FiltersSources
    FiltersExtraction
    GUISupportQt
    IOLegacy
    IOInfovis
    InfovisCore
    InfovisLayout
    InteractionStyle
    RenderingLabel
    RenderingCore
    ViewsInfovis
    CommonColor
    CommonComputationalGeometry
    CommonCore
    CommonDataModel
    RenderingContextOpenGL2
    RenderingCore
    RenderingFreeType
    RenderingGL2PSOpenGL2
    RenderingOpenGL2
    RenderingAnnotation
    RenderingContextOpenGL2
    RenderingGL2PSOpenGL2
    RenderingLOD
)
if(VTK_FOUND)
    message(STATUS "VTK Libraries: ${VTK_LIBRARIES}")
    include(${VTK_USE_FILE})
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

    set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fopenmp -Wno-deprecated -O0")
    find_library(MATHGEO_LIB NAMES MATHGEOLIB MathGeoLib mathgeolib MATHGEO MathGeo mathGeo mathgeo)
    find_path(MATHGEOLIB_H MathGeoLib-master/src)
    find_path(EIGEN_H Eigen)
    find_package(RapidJSON)
    find_path(RAPIDJSON_H rapidjson-master/include/)
    find_library(ORTOOLS_LIB NAMES or-tools ortools Ortools Or-tools OR-TOOLS ORTOOLS)
    find_path(ORTOOLS_H ortools)
#    find_library(SCIP_LIB scip Scip SCIP )
#    find_path(SCIP_H scip/include Scip/include SCIP/include)

    add_subdirectory(${IMATISTLEX})
    add_subdirectory(${ANNOTATION})
    add_subdirectory(${DEFORMATION})
    add_subdirectory(${SLICER})
    add_subdirectory(${SHAPEOP})
    add_subdirectory(${TRIANGLE})
    add_subdirectory(${DATA_STRUCTURES})


    #set( NONRIGID_REGISTRATION ${MAINFOLDER}/NonRigidRegistration/ )
    #add_subdirectory(${NONRIGID_REGISTRATION})


    # Set your files and resources here
    file(GLOB Srcs "${MAINFOLDER}/src/*.c*" "${MAINFOLDER}/src/Styles/*.c*")
    file(GLOB TransferSrcs "${MAINFOLDER}/src/*.c*" "${MAINFOLDER}/src/Styles/*.c*")
    file(GLOB TestCageVsModelSrcs "${MAINFOLDER}/src/*.c*" "${MAINFOLDER}/src/Styles/*.c*")
    file(GLOB TestComputeCoordsSrcs "${MAINFOLDER}/src/*.c*" "${MAINFOLDER}/src/Styles/*.c*")
    file(GLOB TestConstraintsDefinitionSrcs "${MAINFOLDER}/src/*.c*" "${MAINFOLDER}/src/Styles/*.c*")
    file(GLOB Hdrs "${MAINFOLDER}/include/*.h*" "${MAINFOLDER}/include/Styles/*.h*")
    file(GLOB UIs "${MAINFOLDER}/gui/*.ui")

    list(REMOVE_ITEM Srcs
        "${MAINFOLDER}/src/transfer.cpp"
        "${MAINFOLDER}/src/testCageVsModel.cpp"
        "${MAINFOLDER}/src/TestCoordsComputation.cpp"
        "${MAINFOLDER}/src/TestConstraintsDefinition.cpp"
    )
    list(REMOVE_ITEM TransferSrcs
        "${MAINFOLDER}/src/main.cxx"
        "${MAINFOLDER}/src/testCageVsModel.cpp"
        "${MAINFOLDER}/src/TestCoordsComputation.cpp"
        "${MAINFOLDER}/src/TestConstraintsDefinition.cpp"
    )

    list(REMOVE_ITEM TestCageVsModelSrcs
        "${MAINFOLDER}/src/transfer.cpp"
        "${MAINFOLDER}/src/main.cxx"
        "${MAINFOLDER}/src/TestCoordsComputation.cpp"
        "${MAINFOLDER}/src/TestConstraintsDefinition.cpp"
    )

    list(REMOVE_ITEM TestComputeCoordsSrcs
        "${MAINFOLDER}/src/transfer.cpp"
        "${MAINFOLDER}/src/main.cxx"
        "${MAINFOLDER}/src/testCageVsModel.cpp"
        "${MAINFOLDER}/src/TestConstraintsDefinition.cpp"
    )

    list(REMOVE_ITEM TestConstraintsDefinitionSrcs
        "${MAINFOLDER}/src/transfer.cpp"
        "${MAINFOLDER}/src/main.cxx"
        "${MAINFOLDER}/src/testCageVsModel.cpp"
        "${MAINFOLDER}/src/TestCoordsComputation.cpp"
    )

    set( Docs ${CMAKE_CURRENT_SOURCE_DIR}/doc/Doxyfile.in)

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
    add_executable(${PROJECT_NAME} ${Srcs} ${Docs} ${Hdrs} ${UI_Srcs} ${MOC_Hdrs} ${QRC_Srcs} )


    qt5_use_modules(${PROJECT_NAME} Core Gui Widgets)
    target_link_libraries(${PROJECT_NAME} #ConstraintsLib-lib
        Annotation-lib
        ImatiSTLExtension-lib
        Slicer-lib
        Deformation-lib
        DataStructures-lib
        Triangle-lib
        ShapeOp
        ${VTK_LIBRARIES}
        ${MATHGEO_LIB}
        ${ORTOOLS_LIB}
#        ${SCIP_LIB}
    )
    target_include_directories(${PROJECT_NAME} PRIVATE
      ${CMAKE_CURRENT_BINARY_DIR}
      ${CMAKE_CURRENT_SOURCE_DIR}
      ${RAPIDJSON_H}
      ${MATHGEOLIB_H}
      ${EIGEN_H}
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
      ${ORTOOLS_H}/
      ${SCIP_H}/
    )


    find_package(Doxygen)
    if (DOXYGEN_FOUND)
        # set input and output files
        set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/doc/Doxyfile.in)
        set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

        # request to configure the file
        configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)
        message("Doxygen build started")

        # note the option ALL which allows to build the docs together with the application
        add_custom_target( doc_doxygen ALL
            COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
            COMMENT "Generating API documentation with Doxygen"
            VERBATIM )
    else (DOXYGEN_FOUND)
      message("Doxygen need to be installed to generate the doxygen documentation")
    endif (DOXYGEN_FOUND)



    if(${BUILD_TEST_GLPK})
        add_executable(TestGLPK "${MAINFOLDER}/LP/TestGLPK.cpp")
        target_link_libraries(TestGLPK -lglpk
            ${VTK_LIBRARIES})
    endif()

    if(${BUILD_NON_RIGID_REGISTRATION})
        add_executable(TestNonRigidRegistration "${MAINFOLDER}/LP/non_rigid_registration.cpp")
        target_link_libraries(TestNonRigidRegistration ${ORTOOLS_LIB})
        target_include_directories(TestNonRigidRegistration PRIVATE ${ORTOOLS_H}
            ${VTK_LIBRARIES})
    endif()

    if(${BUILD_ANNOTATION_TRANSFER})
        add_executable(AnnotationTransfer ${TransferSrcs} ${Hdrs} ${UI_Srcs} ${MOC_Hdrs} ${QRC_Srcs} )
        qt5_use_modules(AnnotationTransfer Core Gui Widgets)
        target_link_libraries(AnnotationTransfer
            Annotation-lib
            ImatiSTLExtension-lib
            Slicer-lib
            Deformation-lib
            DataStructures-lib
            Triangle-lib
            ShapeOp
            ${VTK_LIBRARIES}
            ${MATHGEO_LIB}
            ${TINYXML2_LIB}
            ${ORTOOLS_LIB}
        )
        target_include_directories(AnnotationTransfer PRIVATE
          ${CMAKE_CURRENT_BINARY_DIR}
          ${CMAKE_CURRENT_SOURCE_DIR}
          ${RAPIDJSON_H}
          ${TINYXML2_H}
          ${MATHGEOLIB_H}
          ${EIGEN_H}
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
          ${ORTOOLS_H}/
        )
    endif()


    if(${BUILD_TEST_CAGE_VS_MODEL})
        add_executable(TestCageVsModel ${TestCageVsModelSrcs} ${Hdrs} ${UI_Srcs} ${MOC_Hdrs} ${QRC_Srcs} )
        qt5_use_modules(TestCageVsModel Core Gui Widgets)
        target_link_libraries(TestCageVsModel
            Annotation-lib
            ImatiSTLExtension-lib
            Slicer-lib
            Deformation-lib
            DataStructures-lib
            Triangle-lib
            ShapeOp
            ${VTK_LIBRARIES}
            ${MATHGEO_LIB}
            ${TINYXML2_LIB}
            ${ORTOOLS_LIB}
        )
        target_include_directories(TestCageVsModel PRIVATE
          ${CMAKE_CURRENT_BINARY_DIR}
          ${CMAKE_CURRENT_SOURCE_DIR}
          ${RAPIDJSON_H}
          ${TINYXML2_H}
          ${MATHGEOLIB_H}
          ${EIGEN_H}
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
          ${ORTOOLS_H}/
        )
    endif()

    if(${BUILD_TEST_COMPUTE_COORDS})
        add_executable(TestComputeCoords ${TestComputeCoordsSrcs} ${Hdrs} ${UI_Srcs} ${MOC_Hdrs} ${QRC_Srcs} )
        qt5_use_modules(TestComputeCoords Core Gui Widgets)
        target_link_libraries(TestComputeCoords
            Annotation-lib
            ImatiSTLExtension-lib
            Slicer-lib
            Deformation-lib
            DataStructures-lib
            Triangle-lib
            ShapeOp
            ${VTK_LIBRARIES}
            ${MATHGEO_LIB}
            ${ORTOOLS_LIB}
        )
        target_include_directories(TestComputeCoords PRIVATE
          ${CMAKE_CURRENT_BINARY_DIR}
          ${CMAKE_CURRENT_SOURCE_DIR}
          ${RAPIDJSON_H}
          ${MATHGEOLIB_H}
          ${EIGEN_H}
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
          ${ORTOOLS_H}/
        )
    endif()

    if(${BUILD_TEST_CONSTRAINTS_DEFINITION})
        add_executable(TestConstraintsDefinition ${TestConstraintsDefinitionSrcs} ${Hdrs} ${UI_Srcs} ${MOC_Hdrs} ${QRC_Srcs} )
        qt5_use_modules(TestConstraintsDefinition Core Gui Widgets)
        target_link_libraries(TestConstraintsDefinition
            Annotation-lib
            ImatiSTLExtension-lib
            Slicer-lib
            Deformation-lib
            DataStructures-lib
            Triangle-lib
            ShapeOp
            ${VTK_LIBRARIES}
            ${MATHGEO_LIB} 
            ${ORTOOLS_LIB}
        )
        target_include_directories(TestConstraintsDefinition PRIVATE
          ${CMAKE_CURRENT_BINARY_DIR}
          ${CMAKE_CURRENT_SOURCE_DIR}
          ${RAPIDJSON_H}
          ${MATHGEOLIB_H}
          ${EIGEN_H}
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
          ${ORTOOLS_H}/
        )
    endif()

endif()
