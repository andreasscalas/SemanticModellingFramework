file(GLOB_RECURSE SRCS
    *.cpp *.h
)
add_library(graphene_surface_mesh_eigen_algorithms STATIC ${SRCS})

target_link_libraries(graphene_surface_mesh_eigen_algorithms
  graphene_surface_mesh
  graphene_geometry
  ${OPENMP_LIBRARIES}
)
