/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_SDL_IMAGE_BUFFER_H
#define ROS_SDL_IMAGE_BUFFER_H

#include <SDL2/SDL.h>
#include <SDL2/SDL_surface.h>
#include <core/Common.h>
#include <core/Environment.h>
#include <graphics/ImageBuffer.h>

namespace ros {
    class SDLImageBuffer: public ImageBuffer {
        public:
            SDLImageBuffer();
            virtual ~SDLImageBuffer();

            SDLImageBuffer(const SDLImageBuffer& src);
            SDLImageBuffer& operator=(const SDLImageBuffer& src);

            virtual bool allocate(U32 width, U32 height, PixelFormat format);
            virtual bool assign(const ImageBuffer& src);
            virtual bool resize(U32 width, U32 height, BlitMode mode);
            virtual bool convert(PixelFormat format);
            virtual void free();

            virtual U32 getWidth() const;
            virtual U32 getHeight() const;
            virtual PixelFormat getFormat() const;
            virtual U32 getBitsPerPixel() const;
            virtual U32 getPitch() const;

            virtual bool isNull() const { return !surface; }
            virtual U32 getSize() const;

            virtual bool clear();

            virtual void* lock();
            virtual const void* lock() const;
            virtual void unlock() const;

        private:
            SDL_Surface* surface;
    };
}

#endif // ROS_SDL_IMAGE_BUFFER_H

