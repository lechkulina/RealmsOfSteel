#
# Copyright (c) 2016 Lech Kulina
#
# This file is part of the Realms Of Steel.
# For conditions of distribution and use, see copyright details in the LICENSE file.
#
set(ROS_VERBOSE_BUILD YES)
set(ROS_BUILD_TYPE DEBUG) # DEBUG / RELEASE
set(ROS_LINKAGE_TYPE SHARED) # SHARED / STATIC
set(ROS_PREFIX_PATH C:/MinGW/msys/1.0 C:/MinGW/msys/1.0/local)
set(ROS_INSTALL_PREFIX C:/deploy/RealmsOfSteel)

set(ROS_IDE_INCLUDES *.h *.hpp *.c *.cpp *.glsl *.lua *.xml *.md *.txt *.cmake*)
set(ROS_IDE_EXCLUDES build)

set(ROS_USE_SDL YES)
set(ROS_USE_SDL_IMAGE YES)
set(ROS_USE_OPENGL YES)
set(ROS_USE_ZIP YES)
