include (ACGCommon)

include_directories (
  ../../..
  ${CMAKE_CURRENT_SOURCE_DIR}
)

acg_add_executable (mconvert mconvert.cc)

target_link_libraries (mconvert
  OpenMeshCore
  OpenMeshTools
)

