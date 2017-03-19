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
        glm::vec3 position;
        glm::vec3 normal;
        glm::vec3 tangent;
        glm::vec3 bitangent;
        glm::vec2 textureCoordinates;
        glm::vec4 color;
    };

    typedef std::vector<Vertex> VertexVector;
    typedef std::vector<U32> IndexVector;
}

#endif // ROS_VERTEX_H

