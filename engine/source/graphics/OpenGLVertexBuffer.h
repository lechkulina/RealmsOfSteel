/*
 * Copyright (c) 2017 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_OPENGL_VERTEX_BUFFER_H
#define ROS_OPENGL_VERTEX_BUFFER_H

#include <GL/glew.h>
#include <graphics/VertexBuffer.h>

namespace ros {
    class OpenGLVertexBuffer: public VertexBuffer {
        public:
            OpenGLVertexBuffer();
            virtual ~OpenGLVertexBuffer();

            virtual bool create();
            virtual bool bind();
            virtual void unbind();
            virtual bool allocate(U32 size, void* data = ROS_NULL, VertexBufferUsage usage = VertexBufferUsage_StaticDraw);
            virtual void free();
            virtual bool isAllocated() const;
            virtual U32 getSize() const { return size; }
            virtual VertexBufferUsageOpt getUsage() const { return usage; }

            virtual void* map(VertexBufferAccess access = VertexBufferAccess_Write);
            virtual void unmap();

        private:
            GLuint handle;
            U32 size;
            VertexBufferUsageOpt usage;
    };
}

#endif // ROS_OPENGL_VERTEX_BUFFER_H

