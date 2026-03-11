add_executable(cmis_orca_smoke_test
  smoke_test.cpp
)

target_link_libraries(cmis_orca_smoke_test
  PRIVATE
    cmis_orca
)

add_test(NAME cmis_orca_smoke_test COMMAND cmis_orca_smoke_test)
