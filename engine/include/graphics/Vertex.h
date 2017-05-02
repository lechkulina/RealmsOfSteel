/*
 * Copyright (c) 2017 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_VERTEX_H
#define ROS_VERTEX_H

#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>
#include <core/Common.h>
#include <core/Environment.h>

namespace ros {
    struct ROS_API Vertex {
        static const U32 MAX_COLORS = 3;
        static const U32 MAX_TEXTURE_COORDS = 3;

        glm::vec4 colors[MAX_COLORS];
        glm::vec3 textureCoords[MAX_TEXTURE_COORDS];
        glm::vec3 position;
        glm::vec3 normal;
        glm::vec3 tangent;
        glm::vec3 bitangent;
    };

    typedef std::vector<Vertex> VerticesVector;
}

#endif // ROS_VERTEX_H

