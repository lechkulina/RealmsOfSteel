#
# Copyright (c) 2016 Lech Kulina
#
# This file is part of the Realms Of Steel.
# For conditions of distribution and use, see copyright details in the LICENSE file.
#
add_library(Engine ${ROS_LINKAGE_TYPE}
    source/core/File.cpp
    source/core/RawBuffer.cpp
    source/core/RawFile.cpp
    source/application/LogEntry.cpp
    source/application/LogsFilter.cpp
    source/application/LogsLevelFilter.cpp
    source/application/LogsSink.cpp
    source/application/LogsConsoleSink.cpp
    source/application/LogsFileSink.cpp
    source/application/Logger.cpp
    source/application/Application.cpp
    source/resources/ArchiveFile.cpp
    source/resources/FileSystem.cpp
    source/resources/ArchiveFileSystem.cpp
    source/resources/Resource.cpp
    source/resources/ResourceLoader.cpp
    source/resources/RawBufferLoader.cpp
    source/resources/ResourcesCache.cpp
    source/resources/StaticResourcesCache.cpp
    source/math/Scalar.cpp
    source/math/Vector2D.cpp
    source/math/Vector3D.cpp
    source/math/Vector4D.cpp
    source/math/Matrix2D.cpp
    source/math/Matrix3D.cpp
    source/math/Matrix4D.cpp
    source/math/Quaternion.cpp
    source/math/EulerAngles.cpp
    source/graphics/Shader.cpp
    source/graphics/Program.cpp
    source/graphics/VertexBuffer.cpp
    source/graphics/IndexBuffer.cpp
    source/graphics/VertexArray.cpp
    source/graphics/Texture.cpp
    source/graphics/Material.cpp
    source/graphics/Mesh.cpp
    source/graphics/SceneNode.cpp
    source/graphics/Scene.cpp
)
target_compile_definitions(Engine
    PUBLIC ROS_VERSION="${PROJECT_VERSION}"
    PUBLIC ROS_LINKAGE_${ROS_LINKAGE_TYPE}
)
target_compile_options(Engine PUBLIC -Wall -Wextra -Wpedantic -Wno-long-long $<$<CONFIG:RELEASE>:-Winline> $<$<CONFIG:RELEASE>:-Wdisabled-optimization>)
target_include_directories(Engine PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

find_package(Boost REQUIRED COMPONENTS system thread filesystem chrono regex)
if (Boost_FOUND)
    target_include_directories(Engine PUBLIC ${Boost_INCLUDE_DIRS})
    target_link_libraries(Engine PUBLIC ${Boost_LIBRARIES})
endif()

find_package(glm REQUIRED)
if (glm_FOUND)
    target_include_directories(Engine PUBLIC ${glm_INCLUDE_DIRS})
endif()

if (ROS_USE_ZIP)
    find_package(ZLIB)
    if (ZLIB_FOUND)
        target_include_directories(Engine PRIVATE ${ZLIB_INCLUDE_DIRS})
        target_link_libraries(Engine PRIVATE ${ZLIB_LIBRARIES})
        target_compile_definitions(Engine PRIVATE ROS_USING_ZIP)
        target_sources(Engine PRIVATE
            source/resources/ZIPArchiveFile.cpp
            source/resources/ZIPArchiveEntry.cpp
        )
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
            source/graphics/OpenGLVertexBuffer.cpp
            source/graphics/OpenGLIndexBuffer.cpp
            source/graphics/OpenGLVertexArray.cpp
        )
    endif()
endif()

if (ROS_USE_SDL)
    find_package(SDL2 REQUIRED)
    if (SDL2_FOUND)
        target_include_directories(Engine PRIVATE ${SDL2_INCLUDE_DIR})
        target_link_libraries(Engine PRIVATE ${SDL2_LIBRARY})
        target_compile_definitions(Engine PRIVATE ROS_USING_SDL SDL_MAIN_HANDLED)
        target_sources(Engine PRIVATE
            source/graphics/SDLImage.cpp
        )
        if (OPENGL_FOUND AND GLEW_FOUND)
            target_sources(Engine PRIVATE
                source/application/SDLOpenGLApplication.cpp
                source/application/SDLOpenGLWindow.cpp
            )
        endif()
    endif()
    if (ROS_USE_SDL_IMAGE)
        find_package(SDL2_image REQUIRED)
        if (SDL2_IMAGE_FOUND)
            target_include_directories(Engine PRIVATE ${SDL2_IMAGE_INCLUDE_DIRS})
            target_link_libraries(Engine PRIVATE ${SDL2_IMAGE_LIBRARIES})
            target_compile_definitions(Engine PRIVATE ROS_USING_SDL_IMAGE)
            target_sources(Engine PRIVATE
                source/resources/SDLImageLoader.cpp
            )
        endif()
    endif()
endif()

if (ROS_USE_ASSIMP)
    find_package(assimp REQUIRED)
    if (ASSIMP_FOUND)
        target_include_directories(Engine PRIVATE ${ASSIMP_INCLUDE_DIRS})
        target_link_libraries(Engine PRIVATE ${ASSIMP_LIBRARIES})
        target_compile_definitions(Engine PRIVATE ROS_USING_ASSIMP)
        target_sources(Engine PRIVATE
            source/resources/AssimpIOStream.cpp
            source/resources/AssimpIOSystem.cpp
            source/resources/AssimpSceneLoader.cpp
        )
    endif()
endif()

if (ROS_LINKAGE_TYPE STREQUAL SHARED)
    install(TARGETS Engine RUNTIME DESTINATION . COMPONENT runtime)
endif()
