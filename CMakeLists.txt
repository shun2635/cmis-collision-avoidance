cmake_minimum_required(VERSION 3.20)

project(cmis_collision_avoidance LANGUAGES CXX)

option(CMIS_BUILD_EXAMPLES "Build example programs" ON)
option(CMIS_BUILD_TESTS "Build test targets" ON)

add_subdirectory(src)

if(CMIS_BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()

if(CMIS_BUILD_TESTS)
  include(CTest)
  if(BUILD_TESTING)
    add_subdirectory(tests)
  endif()
endif()
