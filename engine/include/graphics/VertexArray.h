/*
 * Copyright (c) 2017 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_VERTEX_ARRAY_H
#define ROS_VERTEX_ARRAY_H

#include <core/Common.h>
#include <core/Environment.h>
#include <graphics/Vertex.h>

namespace ros {
    class VertexArray;
    typedef boost::shared_ptr<VertexArray> VertexArrayPtr;

    class ROS_API VertexArray: public boost::noncopyable {
        public:
            virtual ~VertexArray() {}

            static VertexArrayPtr make();

            virtual bool create() =0;
            virtual bool bind() =0;
            virtual void unbind() =0;
            virtual void free() =0;
            virtual bool isCreated() const =0;

            virtual bool enableAttribute(U32 index, VertexAttributeType type) =0;
            virtual void disableAttribute(U32 index) =0;
    };
}

#endif // ROS_VERTEX_ARRAY_H

