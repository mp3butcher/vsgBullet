# - Config file for the osgBullet package
# It defines the following variables
#  OSGBULLET_INCLUDE_DIRS - include directories for osgBullet
#  OSGBULLET_LIBRARIES    - libraries to link against
#  OSGBULLET_EXECUTABLE   - the bar executable


####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was vsgBulletConfig.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../" ABSOLUTE)

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

# These are IMPORTED targets created by osgWorksLibraryDepends.cmake
set(OSGBULLET_LIBRARIES "osgbCollision;osgbDynamics;osgbInteraction" )

# Compute paths
if(EXISTS "${PACKAGE_PREFIX_DIR}/CMakeCache.txt")
    # In build tree
    include("${PACKAGE_PREFIX_DIR}/lib/osgBulletBuildTreeSettings.cmake")
    # Our library dependencies (contains definitions for IMPORTED targets)
    include("${PACKAGE_PREFIX_DIR}/lib/osgBulletLibraryDepends.cmake")
else()
    set_and_check(OSGBULLET_INCLUDE_DIRS "${PACKAGE_PREFIX_DIR}/include")
    # The osgBullet targets file.
    if(EXISTS "${PACKAGE_PREFIX_DIR}/lib/osgbullet-targets.cmake")
        include("${PACKAGE_PREFIX_DIR}/lib/osgbullet-targets.cmake")
    endif()
endif()

# The location of the UseosgBullet.cmake file.
set(osgBullet_USE_FILE "${PACKAGE_PREFIX_DIR}/lib/UseosgBullet.cmake")

set(OSGBULLET_USE_DOUBLE_PRECISION OFF)
set(OSGBULLET_SHARED OFF)
