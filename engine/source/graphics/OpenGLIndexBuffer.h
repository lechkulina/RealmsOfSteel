/*
 * Copyright (c) 2017 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_OPENGL_INDEX_BUFFER_H
#define ROS_OPENGL_INDEX_BUFFER_H

#include <GL/glew.h>
#include <graphics/IndexBuffer.h>

namespace ros {
    class OpenGLIndexBuffer: public IndexBuffer {
        public:
            OpenGLIndexBuffer();
            virtual ~OpenGLIndexBuffer();

            virtual bool create();
            virtual bool bind();
            virtual void unbind();
            virtual bool allocate(U32 indices, const U32* data = ROS_NULL, IndexBufferUsage usage = IndexBufferUsage_StaticDraw);
            virtual void free();
            virtual bool isAllocated() const;
            virtual U32 getIndices() const { return indices; }
            virtual IndexBufferUsageOpt getUsage() const { return usage; }

            virtual U32* map(IndexBufferAccess access = IndexBufferAccess_Write);
            virtual void unmap();

        private:
            GLuint handle;
            U32 indices;
            IndexBufferUsageOpt usage;
    };
}

#endif // ROS_OPENGL_INDEX_BUFFER_H

