/*
 * Copyright (c) 2017 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_INDEX_BUFFER_H
#define ROS_INDEX_BUFFER_H

#include <core/Common.h>
#include <core/Environment.h>

namespace ros {
    enum IndexBufferUsage {
        IndexBufferUsage_StreamDraw,
        IndexBufferUsage_StreamRead,
        IndexBufferUsage_StreamCopy,
        IndexBufferUsage_StaticDraw,
        IndexBufferUsage_StaticRead,
        IndexBufferUsage_StaticCopy,
        IndexBufferUsage_DynamicDraw,
        IndexBufferUsage_DynamicRead,
        IndexBufferUsage_DynamicCopy
    };
    typedef boost::optional<IndexBufferUsage> IndexBufferUsageOpt;

    enum IndexBufferAccess {
        IndexBufferAccess_Read,
        IndexBufferAccess_Write,
        IndexBufferAccess_ReadWrite
    };
    typedef boost::optional<IndexBufferAccess> IndexBufferAccessOpt;

    class IndexBuffer;
    typedef boost::shared_ptr<IndexBuffer> IndexBufferPtr;

    class ROS_API IndexBuffer: public boost::noncopyable {
        public:
            virtual ~IndexBuffer() {}

            static IndexBufferPtr make();

            virtual bool create() =0;
            virtual bool bind() =0;
            virtual void unbind() =0;
            virtual bool allocate(U32 indices, U32* data = ROS_NULL, IndexBufferUsage usage = IndexBufferUsage_StaticDraw) =0;
            virtual void free() =0;
            virtual bool isAllocated() const =0;
            virtual U32 getIndices() const  =0;
            virtual IndexBufferUsageOpt getUsage() const =0;

            virtual U32* map(IndexBufferAccess access = IndexBufferAccess_Write) =0;
            virtual void unmap() =0;
    };
}

#endif // ROS_INDEX_BUFFER_H

