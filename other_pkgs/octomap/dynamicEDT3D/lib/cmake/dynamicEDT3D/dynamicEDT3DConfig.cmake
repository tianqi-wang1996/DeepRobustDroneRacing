# - Config file for the dynamicEDT3D package
#
#  Usage from an external project:
#    In your CMakeLists.txt, add these lines:
#
#    FIND_PACKAGE(dynamicedt3d REQUIRED )
#    INCLUDE_DIRECTORIES(${DYNAMICEDT3D_INCLUDE_DIRS})
#    TARGET_LINK_LIBRARIES(MY_TARGET_NAME ${DYNAMICEDT3D_LIBRARIES})
#
# It defines the following variables
#  DYNAMICEDT3D_INCLUDE_DIRS  - include directories for dynamicEDT3D
#  DYNAMICEDT3D_LIBRARY_DIRS  - library directories for dynamicEDT3D (normally not used!)
#  DYNAMICEDT3D_LIBRARIES     - libraries to link against
#  DYNAMICEDT3D_MAJOR_VERSION - major version
#  DYNAMICEDT3D_MINOR_VERSION - minor version
#  DYNAMICEDT3D_PATCH_VERSION - patch version
#  DYNAMICEDT3D_VERSION       - major.minor.patch version


####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was dynamicEDT3DConfig.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../" ABSOLUTE)

macro(set_and_check _var _file)
  set(${_var} "${_file}")
  if(NOT EXISTS "${_file}")
    message(FATAL_ERROR "File or directory ${_file} referenced by variable ${_var} does not exist !")
  endif()
endmacro()

macro(check_required_components _NAME)
  foreach(comp ${${_NAME}_FIND_COMPONENTS})
    if(NOT ${_NAME}_${comp}_FOUND)
      if(${_NAME}_FIND_REQUIRED_${comp})
        set(${_NAME}_FOUND FALSE)
      endif()
    endif()
  endforeach()
endmacro()

####################################################################################

set(DYNAMICEDT3D_MAJOR_VERSION "1")
set(DYNAMICEDT3D_MINOR_VERSION "9")
set(DYNAMICEDT3D_PATCH_VERSION "4")
set(DYNAMICEDT3D_VERSION "1.9.4")

# Tell the user project where to find our headers and libraries
set_and_check(DYNAMICEDT3D_INCLUDE_DIRS "/home/control02/drone_ws_test/src/octomap/dynamicEDT3D/include")
set_and_check(DYNAMICEDT3D_LIBRARY_DIRS "/home/control02/drone_ws_test/src/octomap/dynamicEDT3D/lib")

set(DYNAMICEDT3D_LIBRARIES "/home/control02/drone_ws_test/src/octomap/dynamicEDT3D/lib/libdynamicedt3d.so")


