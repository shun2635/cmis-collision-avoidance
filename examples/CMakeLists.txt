add_executable(cmis_orca_minimal
  minimal_cli.cpp
)

target_link_libraries(cmis_orca_minimal
  PRIVATE
    cmis_orca
)
