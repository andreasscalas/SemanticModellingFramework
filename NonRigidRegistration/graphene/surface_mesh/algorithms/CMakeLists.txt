file(GLOB_RECURSE SRCS
    *.cpp *.h
)

# opencv needed by templatefitting
# (for windows: make sure to set system environment variable OPENCV_DIR to "path/to/opencv/opencv/build/")
find_package(OpenCV REQUIRED core imgproc objdetect highgui photo)

add_library(graphene_surface_mesh_algorithms STATIC ${SRCS} ${HDRS})

target_link_libraries(graphene_surface_mesh_algorithms
  graphene_surface_mesh
  tinyxml
  graphene_geometry
  graphene_surface_mesh_eigen_algorithms
  ${OpenCV_LIBS}
  ${OPENMP_LIBRARIES}
  dlib::dlib
)

