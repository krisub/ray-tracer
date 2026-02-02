CMAKE_MINIMUM_REQUIRED(VERSION 3.16)
cmake_policy(SET CMP0054 NEW)
set(OpenGL_GL_PREFERENCE "LEGACY")

IF(WIN32)
	# VCPKG
	if(DEFINED ENV{VCPKG_ROOT} AND NOT DEFINED CMAKE_TOOLCHAIN_FILE)
		set(CMAKE_TOOLCHAIN_FILE "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake" CACHE STRING "")
	endif()
ENDIF()

project(GLSL)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

FILE(GLOB cmakes ${CMAKE_SOURCE_DIR}/cmake/*.cmake)

FOREACH(cmake ${cmakes})
	INCLUDE(${cmake})
ENDFOREACH(cmake)

MESSAGE(STATUS "stdgl: ${stdgl_libraries}")

ADD_SUBDIRECTORY(src)

IF(EXISTS ${CMAKE_SOURCE_DIR}/sln/CMakeLists.txt)
	ADD_SUBDIRECTORY(sln)
ENDIF()
