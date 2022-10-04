#----------------------------------------------------------------
# Generated CMake target import file for configuration "Debug".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "3laws::AISupervisor" for configuration "Debug"
set_property(TARGET 3laws::AISupervisor APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(3laws::AISupervisor PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_DEBUG "CXX"
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/libAISupervisor.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS 3laws::AISupervisor )
list(APPEND _IMPORT_CHECK_FILES_FOR_3laws::AISupervisor "${_IMPORT_PREFIX}/lib/libAISupervisor.a" )

# Import target "3laws::AISupervisor-shared" for configuration "Debug"
set_property(TARGET 3laws::AISupervisor-shared APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(3laws::AISupervisor-shared PROPERTIES
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/libAISupervisor-shared.so"
  IMPORTED_SONAME_DEBUG "libAISupervisor-shared.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS 3laws::AISupervisor-shared )
list(APPEND _IMPORT_CHECK_FILES_FOR_3laws::AISupervisor-shared "${_IMPORT_PREFIX}/lib/libAISupervisor-shared.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
