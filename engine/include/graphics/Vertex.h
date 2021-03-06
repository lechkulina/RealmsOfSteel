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
    enum VertexAttributeType {
        VertexAttributeType_Vec2,
        VertexAttributeType_Vec3,
        VertexAttributeType_Vec4
    };

    struct ROS_API Vertex {
        glm::vec4 color;
        glm::vec3 position;
        glm::vec3 normal;
        glm::vec3 tangent;
        glm::vec3 bitangent;
        glm::vec2 textureCoords;

        void setColor(const glm::vec4& color) { this->color = color; }
    };

    typedef std::vector<Vertex> VerticesVector;
}

#endif // ROS_VERTEX_H

