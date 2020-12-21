
file(GLOB children *)


 FOREACH(child ${children})
   IF(IS_DIRECTORY ${child})
   message(STATUS "Added subdirectory ${child}")
   INCLUDE_DIRECTORIES("${child}")
   ADD_SUBDIRECTORY("${child}")
   ENDIF()
 ENDFOREACH()
