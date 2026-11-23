#----------------------------------------------------------------
# Generated CMake target import file for configuration "None".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "liblzf::liblzf" for configuration "None"
set_property(TARGET liblzf::liblzf APPEND PROPERTY IMPORTED_CONFIGURATIONS NONE)
set_target_properties(liblzf::liblzf PROPERTIES
  IMPORTED_LOCATION_NONE "${_IMPORT_PREFIX}/lib/x86_64-linux-gnu/liblzf.so.1.5"
  IMPORTED_SONAME_NONE "liblzf.so.1"
  )

list(APPEND _IMPORT_CHECK_TARGETS liblzf::liblzf )
list(APPEND _IMPORT_CHECK_FILES_FOR_liblzf::liblzf "${_IMPORT_PREFIX}/lib/x86_64-linux-gnu/liblzf.so.1.5" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
