#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "3laws::AISupervisor" for configuration "Release"
set_property(TARGET 3laws::AISupervisor APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(3laws::AISupervisor PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libAISupervisor.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS 3laws::AISupervisor )
list(APPEND _IMPORT_CHECK_FILES_FOR_3laws::AISupervisor "${_IMPORT_PREFIX}/lib/libAISupervisor.a" )

# Import target "3laws::AISupervisor-shared" for configuration "Release"
set_property(TARGET 3laws::AISupervisor-shared APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(3laws::AISupervisor-shared PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libAISupervisor-shared.so"
  IMPORTED_SONAME_RELEASE "libAISupervisor-shared.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS 3laws::AISupervisor-shared )
list(APPEND _IMPORT_CHECK_FILES_FOR_3laws::AISupervisor-shared "${_IMPORT_PREFIX}/lib/libAISupervisor-shared.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
