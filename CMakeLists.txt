#
# Copyright (c) 2016 Lech Kulina
#
# This file is part of the Realms Of Steel.
# For conditions of distribution and use, see copyright details in the LICENSE file.
#
cmake_minimum_required(VERSION 3.0)
project(RealmsOfSteel VERSION 0.0.1)

include(Config.cmake)

if (ROS_VERBOSE_BUILD)
    set(CMAKE_VERBOSE_MAKEFILE YES)
    set(CMAKE_INSTALL_MESSAGE ALWAYS)
else()
    set(CMAKE_VERBOSE_MAKEFILE NO)
    set(CMAKE_INSTALL_MESSAGE NEVER)
endif()
set(CMAKE_SHARED_LIBRARY_PREFIX)
set(CMAKE_DEBUG_POSTFIX)
set(CMAKE_PREFIX_PATH ${ROS_PREFIX_PATH})
set(CMAKE_BUILD_TYPE ${ROS_BUILD_TYPE})
set(CMAKE_INSTALL_PREFIX ${ROS_INSTALL_PREFIX})

file(GLOB_RECURSE ROS_IDE_FILES RELATIVE ${CMAKE_SOURCE_DIR} ${ROS_IDE_INCLUDES})
foreach(ROS_IDE_FILE IN LISTS ROS_IDE_FILES)
    string(REGEX MATCH ${ROS_IDE_EXCLUDES} ROS_IDE_MATCHES ${ROS_IDE_FILE})
    if (ROS_IDE_MATCHES)
        list(REMOVE_ITEM ROS_IDE_FILES ${ROS_IDE_FILE})
    endif()
    unset(ROS_IDE_MATCHES)
endforeach()
add_custom_target(IDE ALL SOURCES ${ROS_IDE_FILES})
unset(ROS_IDE_FILES)

add_subdirectory(Engine)
add_subdirectory(Game)

install(FILES README.md LICENSE DESTINATION . COMPONENT runtime)
