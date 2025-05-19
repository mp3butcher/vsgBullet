#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "osgbCollision" for configuration "Release"
set_property(TARGET osgbCollision APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(osgbCollision PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libosgbCollision.a"
  )

list(APPEND _cmake_import_check_targets osgbCollision )
list(APPEND _cmake_import_check_files_for_osgbCollision "${_IMPORT_PREFIX}/lib/libosgbCollision.a" )

# Import target "osgbDynamics" for configuration "Release"
set_property(TARGET osgbDynamics APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(osgbDynamics PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libosgbDynamics.a"
  )

list(APPEND _cmake_import_check_targets osgbDynamics )
list(APPEND _cmake_import_check_files_for_osgbDynamics "${_IMPORT_PREFIX}/lib/libosgbDynamics.a" )

# Import target "osgbInteraction" for configuration "Release"
set_property(TARGET osgbInteraction APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(osgbInteraction PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libosgbInteraction.a"
  )

list(APPEND _cmake_import_check_targets osgbInteraction )
list(APPEND _cmake_import_check_files_for_osgbInteraction "${_IMPORT_PREFIX}/lib/libosgbInteraction.a" )

# Import target "osgdb_osgbdynamics" for configuration "Release"
set_property(TARGET osgdb_osgbdynamics APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(osgdb_osgbdynamics PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/osgdb_osgbdynamics.a"
  )

list(APPEND _cmake_import_check_targets osgdb_osgbdynamics )
list(APPEND _cmake_import_check_files_for_osgdb_osgbdynamics "${_IMPORT_PREFIX}/lib/osgdb_osgbdynamics.a" )

# Import target "osgdb_sgb" for configuration "Release"
set_property(TARGET osgdb_sgb APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(osgdb_sgb PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/osgdb_sgb.a"
  )

list(APPEND _cmake_import_check_targets osgdb_sgb )
list(APPEND _cmake_import_check_files_for_osgdb_sgb "${_IMPORT_PREFIX}/lib/osgdb_sgb.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
