#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "abb_librws::abb_librws" for configuration ""
set_property(TARGET abb_librws::abb_librws APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(abb_librws::abb_librws PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libabb_librws.dylib"
  IMPORTED_SONAME_NOCONFIG "@rpath/libabb_librws.dylib"
  )

list(APPEND _IMPORT_CHECK_TARGETS abb_librws::abb_librws )
list(APPEND _IMPORT_CHECK_FILES_FOR_abb_librws::abb_librws "${_IMPORT_PREFIX}/lib/libabb_librws.dylib" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
