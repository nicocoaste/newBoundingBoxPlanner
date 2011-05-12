# Olivier Stasse, Francois Keith, Nicolas Perrin JRL, CNRS/AIST
# Creation: 31/10/2008
# History:
#
# Copyright CNRS/AIST

cmake_minimum_required(VERSION 2.6)

enable_language(CXX)

# Debug mode for cmake.
SET(CMAKE_VERBOSE_MAKEFILE ON)

# Set version
SET(PROJECT_VERSION 0.9.99)

IF(WIN32)
  SET(LIBDIR_KW "/LIBPATH:")
  SET(LIBINCLUSION_KW "")
  SET(LIBINCLUSION_EXT ".lib")
ENDIF(WIN32)

IF(UNIX)
  SET(LIBDIR_KW "-L")
  SET(LIBINCLUSION_KW "-l")
  SET(LIBINCLUSION_EXT "")
ENDIF(UNIX)

# name of the project
SET(PROJECT_NAME newBoundingBoxPlanner)
SET(PROJECT_CONTACT )
SET(PROJECT_DESCRIPTION "")
project(${PROJECT_NAME})

#SET(CMAKE_INSTALL_PREFIX "./build/")

# Impose shared libraries
#------------------------
OPTION(BUILD_SHARED_LIBS "Build it with shared libraries." ON)

# Check for required packages
# -------------------------
INCLUDE(FindPkgConfig)

pkg_check_modules(PQP REQUIRED PQP>=1.3)
pkg_check_modules(OMPL REQUIRED OMPL>=0.9)
pkg_check_modules(ASSIMP REQUIRED assimp>=2.0)
pkg_check_modules(NSPG REQUIRED newSliderPG>=0.9)
pkg_check_modules(HRP2R REQUIRED hrp2Robot>=0.9)


# -----

SET(PROJECT_REQUIREMENTS "PQP >= 1.3 OMPL >= 0.9 assimp >= 2.0 newSliderPG >= 0.9 hrp2Robot >= 0.9")

# -----

MESSAGE(STATUS "${PQP_CFLAGS}")
MESSAGE(STATUS "${OMPL_CFLAGS}")
MESSAGE(STATUS "${ASSIMP_CFLAGS}")
MESSAGE(STATUS "${NSPG_CFLAGS}")
MESSAGE(STATUS "${HRP2R_CFLAGS}")

# -----

SET(${PROJECT_NAME}_CXXFLAGS "-Wall ${${PROJECT_NAME}_CXXFLAGS}")

# -----

FOREACH(option_cxxflags ${PQP_CFLAGS})
  SET(${PROJECT_NAME}_CXXFLAGS "${option_cxxflags} ${${PROJECT_NAME}_CXXFLAGS}")
ENDFOREACH(option_cxxflags)

FOREACH(option_cxxflags ${OMPL_CFLAGS})
  SET(${PROJECT_NAME}_CXXFLAGS "${option_cxxflags} ${${PROJECT_NAME}_CXXFLAGS}")
ENDFOREACH(option_cxxflags)

FOREACH(option_cxxflags ${ASSIMP_CFLAGS})
  SET(${PROJECT_NAME}_CXXFLAGS "${option_cxxflags} ${${PROJECT_NAME}_CXXFLAGS}")
ENDFOREACH(option_cxxflags)

FOREACH(option_cxxflags ${NSPG_CFLAGS})
  SET(${PROJECT_NAME}_CXXFLAGS "${option_cxxflags} ${${PROJECT_NAME}_CXXFLAGS}")
ENDFOREACH(option_cxxflags)

FOREACH(option_cxxflags ${HRP2R_CFLAGS})
  SET(${PROJECT_NAME}_CXXFLAGS "${option_cxxflags} ${${PROJECT_NAME}_CXXFLAGS}")
ENDFOREACH(option_cxxflags)

# -----

FOREACH(option_ldflags ${PQP_LDFLAGS})
  SET(${PROJECT_NAME}_LDFLAGS "${option_ldflags} ${${PROJECT_NAME}_LDFLAGS}")
ENDFOREACH(option_ldflags)

FOREACH(option_ldflags ${OMPL_LDFLAGS})
  SET(${PROJECT_NAME}_LDFLAGS "${option_ldflags} ${${PROJECT_NAME}_LDFLAGS}")
ENDFOREACH(option_ldflags)

FOREACH(option_ldflags ${ASSIMP_LDFLAGS})
  SET(${PROJECT_NAME}_LDFLAGS "${option_ldflags} ${${PROJECT_NAME}_LDFLAGS}")
ENDFOREACH(option_ldflags)

FOREACH(option_ldflags ${NSPG_LDFLAGS})
  SET(${PROJECT_NAME}_LDFLAGS "${option_ldflags} ${${PROJECT_NAME}_LDFLAGS}")
ENDFOREACH(option_ldflags)

FOREACH(option_ldflags ${HRP2R_LDFLAGS})
  SET(${PROJECT_NAME}_LDFLAGS "${option_ldflags} ${${PROJECT_NAME}_LDFLAGS}")
ENDFOREACH(option_ldflags)

# -----

SET(${PROJECT_NAME}_src_LDFLAGS ${${PROJECT_NAME}_LDFLAGS})
SET(${PROJECT_NAME}_LDFLAGS "${LIBDIR_KW}${CMAKE_PREFIX_INSTALL}/lib ${LIBINCLUSION_KW}${PROJECT_NAME}-${PROJECT_VERSION}${LIBINCLUSION_EXT} ${${PROJECT_NAME}_LDFLAGS}")

# -------------------------

add_subdirectory(src)
add_subdirectory(unitTesting)

# Prepare the documentation
# -------------------------
INCLUDE(FindDoxygen)
IF(DOXYGEN_FOUND)
  MESSAGE(STATUS "Doxygen found")
  add_subdirectory(doc)
ENDIF(DOXYGEN_FOUND)

# Prepare description file for pkg-config.
#-----------------------------------------

SET(install_pkg_prefix "\${prefix}")
SET(install_pkg_exec_prefix "\${exec_prefix}")
SET(install_pkg_libdir "\${libdir}")
SET(install_pkg_include_dir "\${includedir}")

# Install the file in the appropriate place.
SET(install_libdir_pkgconfig ${CMAKE_INSTALL_PREFIX}/lib/pkgconfig)

CONFIGURE_FILE(${${PROJECT_NAME}_SOURCE_DIR}/${PROJECT_NAME}.pc.cmake
		 ${${PROJECT_NAME}_BINARY_DIR}/${PROJECT_NAME}.pc )

INSTALL(FILES ${${PROJECT_NAME}_BINARY_DIR}/${PROJECT_NAME}.pc
	DESTINATION ${install_libdir_pkgconfig}	
        PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE )
