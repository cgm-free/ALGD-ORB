cmake_minimum_required(VERSION 2.8)
project( orb )

# OpenCV 
# find_package( OpenCV REQUIRED )
find_package(OpenCV QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()


include_directories( 
${PROJECT_SOURCE_DIR}/include
${OpenCV_INCLUDE_DIRS} 
#/home/cgm/ORB_Feature/ORB_Modify/include
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
        src/ORB_modify.cc
        src/gms_matcher.cc
        src/ORBextractor.cc

)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

add_executable( orb_matcher ORB_matcher.cc )
target_link_libraries( orb_matcher ${PROJECT_NAME} )