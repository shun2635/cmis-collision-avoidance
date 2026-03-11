add_library(cmis_orca
  simulator.cpp
)

target_include_directories(cmis_orca
  PUBLIC
    ${PROJECT_SOURCE_DIR}/include
)

target_compile_features(cmis_orca PUBLIC cxx_std_20)

if(MSVC)
  target_compile_options(cmis_orca PRIVATE /W4 /permissive-)
else()
  target_compile_options(cmis_orca PRIVATE -Wall -Wextra -Wpedantic)
endif()
