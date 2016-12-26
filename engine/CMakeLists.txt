#
# Copyright (c) 2016 Lech Kulina
#
# This file is part of the Realms Of Steel.
# For conditions of distribution and use, see copyright details in the LICENSE file.
#
add_library(Engine ${ROS_LINKAGE_TYPE}
    source/application/LogMessage.cpp
    source/application/LogsFilter.cpp
    source/application/LogsLevelFilter.cpp
    source/application/LogsSink.cpp
    source/application/LogsConsoleSink.cpp
    source/application/LogsFileSink.cpp
    source/application/Logger.cpp
    source/application/Application.cpp
    source/math/Scalar.cpp
)
target_compile_definitions(Engine
    PUBLIC ROS_VERSION="${PROJECT_VERSION}"
    PUBLIC ROS_LINKAGE_${ROS_LINKAGE_TYPE}
)
target_compile_options(Engine PUBLIC -Wall -Wextra -Wpedantic -Wno-long-long $<$<CONFIG:RELEASE>:-Winline> $<$<CONFIG:RELEASE>:-Wdisabled-optimization>)
target_include_directories(Engine PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

find_package(Boost REQUIRED COMPONENTS system thread filesystem chrono)
if (Boost_FOUND)
    target_include_directories(Engine PUBLIC ${Boost_INCLUDE_DIRS})
    target_link_libraries(Engine PUBLIC ${Boost_LIBRARIES})
endif()

if (ROS_USE_ZIP)
    find_package(ZLIB)
    if (ZLIB_FOUND)
        target_include_directories(Engine PRIVATE ${ZLIB_INCLUDE_DIRS})
        target_link_libraries(Engine PRIVATE ${ZLIB_LIBRARIES})
        target_compile_definitions(Engine PRIVATE ROS_USING_ZIP)
    endif()
endif()

if (ROS_USE_OPENGL)
    find_package(OpenGL REQUIRED)
    find_package(GLEW REQUIRED)
    if (OPENGL_FOUND AND GLEW_FOUND)
        target_include_directories(Engine PRIVATE ${OPENGL_INCLUDE_DIR} ${GLEW_INCLUDE_DIRS})
        target_link_libraries(Engine PRIVATE ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES})
        target_compile_definitions(Engine PRIVATE ROS_USING_OPENGL)
        target_sources(Engine PRIVATE
            source/graphics/OpenGLErrors.cpp
            source/graphics/OpenGLShader.cpp
            source/graphics/OpenGLProgram.cpp
        )
    endif()
endif()

if (ROS_USE_SDL)
    find_package(SDL2 REQUIRED)
    if (SDL2_FOUND)
        target_include_directories(Engine PRIVATE ${SDL2_INCLUDE_DIR})
        target_link_libraries(Engine PRIVATE ${SDL2_LIBRARY})
        target_compile_definitions(Engine PRIVATE ROS_USING_SDL SDL_MAIN_HANDLED)
        if (OPENGL_FOUND AND GLEW_FOUND)
            target_sources(Engine PRIVATE
                source/application/SDLOpenGLApplication.cpp
                source/application/SDLOpenGLWindow.cpp
            )
        endif()
    endif()
endif()

if (ROS_LINKAGE_TYPE STREQUAL SHARED)
    install(TARGETS Engine RUNTIME DESTINATION . COMPONENT runtime)
endif()
