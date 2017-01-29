/*
 * Copyright (c) 2017 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_IMAGE_BUFFER_H
#define ROS_IMAGE_BUFFER_H

#include <core/Common.h>
#include <core/Environment.h>
#include <core/Buffer.h>

namespace ros {
    enum PixelFormat {
        PixelFormat_Unknown,
        PixelFormat_RGB565,
        PixelFormat_BGR565,
        PixelFormat_RGB888,
        PixelFormat_BGR888,
        PixelFormat_ARGB8888,
        PixelFormat_ABGR8888,
        PixelFormat_RGBA8888,
        PixelFormat_BGRA8888
    };

    enum BlitMode {
        BlitMode_Crop,
        BlitMode_Scaled
    };

    class ROS_API ImageBuffer: public Buffer {
        public:
            virtual bool allocate(U32 width, U32 height, PixelFormat format) =0;
            virtual bool assign(const ImageBuffer& src) =0;
            virtual bool resize(U32 width, U32 height, BlitMode mode) =0;
            virtual bool convert(PixelFormat format) =0;
            virtual void free() =0;

            virtual U32 getWidth() const =0;
            virtual U32 getHeight() const =0;
            virtual PixelFormat getFormat() const =0;
            virtual U32 getBitsPerPixel() const =0;
            virtual U32 getPitch() const =0;

            virtual bool clear() =0;

            virtual void* lock() =0;
            virtual const void* lock() const =0;
            virtual void unlock() const =0;
    };

    typedef boost::shared_ptr<ImageBuffer> ImageBufferPtr;
}

#endif // ROS_IMAGE_BUFFER_H

