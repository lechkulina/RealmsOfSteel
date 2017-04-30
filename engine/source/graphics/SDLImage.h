/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_SDL_IMAGE_H
#define ROS_SDL_IMAGE_H

#include <SDL2/SDL.h>
#include <SDL2/SDL_surface.h>
#include <core/Common.h>
#include <core/Environment.h>
#include <graphics/Image.h>

namespace ros {
    class SDLImage : public Image {
        public:
            SDLImage();
            SDLImage(SDL_Surface* src);
            virtual ~SDLImage();

            SDLImage(const SDLImage& src);
            SDLImage& operator=(const SDLImage& src);

            virtual bool allocate(U32 width, U32 height, PixelFormat format);
            virtual bool assign(const Image& src);
            bool assign(SDL_Surface* src);
            virtual bool resize(U32 width, U32 height, BlitMode mode);
            virtual bool convert(PixelFormat format);
            virtual void free();

            virtual U32 getWidth() const;
            virtual U32 getHeight() const;
            virtual PixelFormat getFormat() const;
            virtual U32 getBitsPerPixel() const;
            virtual U32 getPitch() const;

            virtual bool isNull() const { return !surface; }

            virtual bool clear();

            virtual void* lock();
            virtual const void* lock() const;
            virtual void unlock() const;

        private:
            SDL_Surface* surface;
    };

    typedef boost::shared_ptr<SDLImage> SDLImagePtr;
}

#endif // ROS_SDL_IMAGE_H

