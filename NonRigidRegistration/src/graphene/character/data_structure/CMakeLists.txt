file(GLOB SRCS ./*.cpp ./*.c)
file(GLOB HDRS ./*.h )


if(FBX_LIBRARY)
    set(HDRS ${HDRS} fbxfile/fbx_file.h)
    set(SRCS ${SRCS} fbxfile/fbx_file.cpp)
    #add_definitions(-DFBXSDK_SHARED)
endif()

add_library(graphene_character STATIC ${SRCS} ${HDRS})

if(FBX_LIBRARY)
    target_link_libraries(graphene_character graphene_surface_mesh tinyxml ${OPENMP_LIBRARIES} ${FBX_LIBRARY})
else()
    target_link_libraries(graphene_character graphene_surface_mesh tinyxml ${OPENMP_LIBRARIES})
endif()
