file(GLOB_RECURSE SRCS ./*.cpp ./*.c)
file(GLOB_RECURSE HDRS ./*.h)


add_executable(points2head
  ${SRCS} ${HDRS}
)

target_link_libraries(points2head
    pmp
    graphene_gl
    io
    graphene_scene_graph
    graphene_character_scene_graph
    graphene_surface_mesh_algorithms
    graphene_pointset_processing)
