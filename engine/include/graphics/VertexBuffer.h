/*
 * Copyright (c) 2017 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_VERTEX_BUFFER_H
#define ROS_VERTEX_BUFFER_H

#include <core/Common.h>
#include <core/Environment.h>

namespace ros {
    enum VertexBufferUsage {
        VertexBufferUsage_StreamDraw,
        VertexBufferUsage_StreamRead,
        VertexBufferUsage_StreamCopy,
        VertexBufferUsage_StaticDraw,
        VertexBufferUsage_StaticRead,
        VertexBufferUsage_StaticCopy,
        VertexBufferUsage_DynamicDraw,
        VertexBufferUsage_DynamicRead,
        VertexBufferUsage_DynamicCopy
    };
    typedef boost::optional<VertexBufferUsage> VertexBufferUsageOpt;

    enum VertexBufferAccess {
        VertexBufferAccess_Read,
        VertexBufferAccess_Write,
        VertexBufferAccess_ReadWrite
    };
    typedef boost::optional<VertexBufferAccess> VertexBufferAccessOpt;

    class VertexBuffer;
    typedef boost::shared_ptr<VertexBuffer> VertexBufferPtr;

    class ROS_API VertexBuffer: public boost::noncopyable {
        public:
            virtual ~VertexBuffer() {}

            static VertexBufferPtr make();

            virtual bool create() =0;
            virtual bool bind() =0;
            virtual void unbind() =0;
            virtual bool allocate(U32 size, void* data = ROS_NULL, VertexBufferUsage usage = VertexBufferUsage_StaticDraw) =0;
            virtual void free() =0;
            virtual bool isAllocated() const =0;
            virtual U32 getSize() const  =0;
            virtual VertexBufferUsageOpt getUsage() const =0;

            virtual void* map(VertexBufferAccess access = VertexBufferAccess_Write) =0;
            virtual void unmap() =0;
    };
}

#endif // ROS_VERTEX_BUFFER_H

