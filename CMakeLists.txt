cmake_minimum_required(VERSION 2.7 FATAL_ERROR)
project(Corexapod)

include_directories(${PROJECT_SOURCE_DIR}/include)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

set( LIB_SRC
    src/engine.cpp src/serial.cpp
)

add_library( corexapod SHARED ${LIB_SRC} )
#add_definitions( ${SSE_FLAGS} )
target_link_libraries( corexapod )


add_executable( test src/test.cpp )
target_link_libraries( test corexapod )
