file(GLOB_RECURSE SRCS ./*.cpp)
file(GLOB_RECURSE HDRS ./*.h)



add_library(graphene_gl STATIC ${SRCS} ${HDRS})

#target_link_libraries(graphene_gl glew ${OPENGL_LIBRARIES})
target_link_libraries(graphene_gl glew ${OPENGL_LIBRARIES} cl)

file(COPY shaders DESTINATION ${PROJECT_BINARY_DIR})

