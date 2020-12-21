
file(GLOB_RECURSE SRCS ./*.cpp)
file(GLOB_RECURSE HDRS ./*.h)


add_library(graphene_pointset_processing STATIC ${SRCS} ${HDRS} )

target_link_libraries(graphene_pointset_processing)
