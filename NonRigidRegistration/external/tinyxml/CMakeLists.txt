file(GLOB_RECURSE SRCS ./*.cpp ./*.h) 


add_library(tinyxml STATIC ${SRCS})
