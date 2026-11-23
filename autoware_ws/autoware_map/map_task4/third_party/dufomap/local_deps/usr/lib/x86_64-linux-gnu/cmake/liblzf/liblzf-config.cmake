
####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was liblzf-config.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../../" ABSOLUTE)

# Use original install prefix when loaded through a "/usr move"
# cross-prefix symbolic link such as /lib -> /usr/lib.
get_filename_component(_realCurr "${CMAKE_CURRENT_LIST_DIR}" REALPATH)
get_filename_component(_realOrig "/usr/lib/x86_64-linux-gnu/cmake/liblzf" REALPATH)
if(_realCurr STREQUAL _realOrig)
  set(PACKAGE_PREFIX_DIR "/usr")
endif()
unset(_realOrig)
unset(_realCurr)

macro(set_and_check _var _file)
  set(${_var} "${_file}")
  if(NOT EXISTS "${_file}")
    message(FATAL_ERROR "File or directory ${_file} referenced by variable ${_var} does not exist !")
  endif()
endmacro()

####################################################################################

include("${CMAKE_CURRENT_LIST_DIR}/liblzf-targets.cmake")

set_source_files_properties(
	${PACKAGE_PREFIX_DIR}/src/liblzf/lzf_c.c
	${PACKAGE_PREFIX_DIR}/src/liblzf/lzf_d.c
	PROPERTIES
	LANGUAGE "C"
)
if(CMAKE_C_COMPILER_ID STREQUAL "GNU" OR CMAKE_C_COMPILER_ID STREQUAL "Clang")
	set_source_files_properties(
		${PACKAGE_PREFIX_DIR}/src/liblzf/lzf_c.c
		${PACKAGE_PREFIX_DIR}/src/liblzf/lzf_d.c
		PROPERTIES
		COMPILE_OPTIONS "-funroll-all-loops"
	)
endif()

set_and_check(liblzf_INCLUDE_DIRS "${PACKAGE_PREFIX_DIR}/include/liblzf")
set(liblzf_SOURCES "${PACKAGE_PREFIX_DIR}/src/liblzf/lzf_c.c;${PACKAGE_PREFIX_DIR}/src/liblzf/lzf_d.c")
set(liblzf_VERSION_STRING "3.6")
set(liblzf_VERSION_MAJOR "3")
set(liblzf_VERSION_MINOR "6")
set(liblzf_API_VERSION_STRING "1.5")
set(liblzf_API_VERSION_MAJOR "1")
set(liblzf_API_VERSION_MINOR "5")
set(liblzf_FOUND TRUE)
