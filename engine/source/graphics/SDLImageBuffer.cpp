/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <boost/bind.hpp>
#include <boost/range.hpp>
#include <SDL2/SDL_pixels.h>
#include <application/Logger.h>
#include "SDLImageBuffer.h"

namespace {
    const struct PixelFormatMapping {
        ros::PixelFormat format;
        Uint32 native;
    } pixelFormatMappings[] = {
        {ros::PixelFormat_Unknown, SDL_PIXELFORMAT_UNKNOWN},
        {ros::PixelFormat_RGB565, SDL_PIXELFORMAT_RGB565},
        {ros::PixelFormat_BGR565, SDL_PIXELFORMAT_BGR565},
        {ros::PixelFormat_RGB888, SDL_PIXELFORMAT_RGB888},
        {ros::PixelFormat_BGR888, SDL_PIXELFORMAT_BGR888},
        {ros::PixelFormat_ARGB8888, SDL_PIXELFORMAT_ARGB8888},
        {ros::PixelFormat_ABGR8888, SDL_PIXELFORMAT_ABGR8888},
        {ros::PixelFormat_RGBA8888, SDL_PIXELFORMAT_RGBA8888},
        {ros::PixelFormat_BGRA8888, SDL_PIXELFORMAT_BGRA8888}
    };

    ros::PixelFormat PixelFormat_fromSDLFormat(Uint32  native) {
        const PixelFormatMapping* iter = std::find_if(boost::begin(pixelFormatMappings), boost::end(pixelFormatMappings),
            boost::bind(&PixelFormatMapping::native, _1) == native);
        if (iter != boost::end(pixelFormatMappings)) {
            return iter->format;
        }
        return ros::PixelFormat_Unknown;
    }

    Uint32 SDLFormat_fromPixelFormat(ros::PixelFormat format) {
        const PixelFormatMapping* iter = std::find_if(boost::begin(pixelFormatMappings), boost::end(pixelFormatMappings),
            boost::bind(&PixelFormatMapping::format, _1) == format);
        if (iter != boost::end(pixelFormatMappings)) {
            return iter->native;
        }
        return SDL_PIXELFORMAT_UNKNOWN;
    }
}

ros::SDLImageBuffer::SDLImageBuffer()
    : surface(ROS_NULL) {
}

ros::SDLImageBuffer::~SDLImageBuffer() {
    free();
}

ros::SDLImageBuffer::SDLImageBuffer(const SDLImageBuffer& src)
    : surface(ROS_NULL){
    assign(src);
}

ros::SDLImageBuffer& ros::SDLImageBuffer::operator=(const SDLImageBuffer& src) {
    assign(src);
    return *this;
}

bool ros::SDLImageBuffer::allocate(U32 width, U32 height, PixelFormat format) {
    free();

    Uint32 nativeFormat = SDLFormat_fromPixelFormat(format);
    if (nativeFormat == SDL_PIXELFORMAT_UNKNOWN) {
        Logger::report(LogLevel_Error, boost::format("Unsupported pixel format %d") % nativeFormat);
        free();
        return false;
    }

    int depth;
    Uint32 redMask, greenMask, blueMask, alphaMask;
    if (!SDL_PixelFormatEnumToMasks(nativeFormat, &depth, &redMask, &greenMask, &blueMask, &alphaMask)) {
        Logger::report(LogLevel_Error, boost::format("Failed to compute channels masks and depth - SDL error occured %s")  % SDL_GetError());
        free();
        return false;
    }

    surface = SDL_CreateRGBSurface(0, width, height, depth, redMask, greenMask, blueMask, alphaMask);
    if (!surface) {
        Logger::report(LogLevel_Error, boost::format("Failed to create surface - SDL error occured %s") % SDL_GetError());
        free();
        return false;
    }

    return true;
}

bool ros::SDLImageBuffer::assign(const ImageBuffer& src) {
    if (this == &src) {
        return true;
    }
    if (src.isNull()) {
        return false;
    }

    free();

    void* data = const_cast<void*>(src.lock());
    if (!data) {
        return false;
    }

    Uint32 nativeFormat = SDLFormat_fromPixelFormat(src.getFormat());
    if (nativeFormat == SDL_PIXELFORMAT_UNKNOWN) {
        Logger::report(LogLevel_Error, boost::format("Unsupported pixel format %d") % nativeFormat);
        free();
        return false;
    }

    int depth;
    Uint32 redMask, greenMask, blueMask, alphaMask;
    if (!SDL_PixelFormatEnumToMasks(nativeFormat, &depth, &redMask, &greenMask, &blueMask, &alphaMask)) {
        Logger::report(LogLevel_Error, boost::format("Failed to compute channels masks and depth - SDL error occured %s")  % SDL_GetError());
        free();
        return false;
    }

    surface = SDL_CreateRGBSurfaceFrom(data, src.getWidth(), src.getHeight(), depth, src.getPitch(), redMask, greenMask, blueMask, alphaMask);
    if (!surface) {
        Logger::report(LogLevel_Error, boost::format("Failed to create surface - SDL error occured %s") % SDL_GetError());
        free();
        return false;
    }

    src.unlock();
    return true;
}

bool ros::SDLImageBuffer::resize(U32 width, U32 height, BlitMode mode) {
    if (!surface) {
        return false;
    }

    int depth;
    Uint32 redMask, greenMask, blueMask, alphaMask;
    if (!SDL_PixelFormatEnumToMasks(surface->format->format, &depth, &redMask, &greenMask, &blueMask, &alphaMask)) {
        Logger::report(LogLevel_Error, boost::format("Failed to compute channels masks and depth - SDL error occured %s")  % SDL_GetError());
        free();
        return false;
    }

    SDL_Surface* surface = SDL_CreateRGBSurface(0, width, height, depth, redMask, greenMask, blueMask, alphaMask);
    if (!surface) {
        Logger::report(LogLevel_Error, boost::format("Failed to create surface - SDL error occured %s") % SDL_GetError());
        return false;
    }

    SDL_Rect dimensions = {0, 0, static_cast<int>(width), static_cast<int>(height)};
    switch (mode) {
        case BlitMode_Crop:
            if (SDL_BlitSurface(this->surface, &dimensions, surface, ROS_NULL) != 0) {
                Logger::report(LogLevel_Error, boost::format("Failed to blit a surface - SDL error occured %s") % SDL_GetError());
                SDL_FreeSurface(surface);
                return false;
            }
            break;
        case BlitMode_Scaled:
            if (SDL_BlitScaled(this->surface, ROS_NULL, surface, &dimensions) != 0) {
                Logger::report(LogLevel_Error, boost::format("Failed to blit a surface - SDL error occured %s") % SDL_GetError());
                SDL_FreeSurface(surface);
                return false;
            }
            break;
        default:
            Logger::report(LogLevel_Error, boost::format("Unsupported blit mode %d") % mode);
            SDL_FreeSurface(surface);
            return false;
    }

    free();
    this->surface = surface;

    return true;
}

bool ros::SDLImageBuffer::convert(PixelFormat format) {
    if (!surface) {
        return false;
    }

    Uint32 nativeFormat = SDLFormat_fromPixelFormat(format);
    if (nativeFormat == SDL_PIXELFORMAT_UNKNOWN) {
        Logger::report(LogLevel_Error, boost::format("Unsupported pixel format %d") % nativeFormat);
        return false;
    }
    SDL_Surface* surface = SDL_ConvertSurfaceFormat(this->surface, nativeFormat, 0);
    if (!surface) {
        Logger::report(LogLevel_Error, boost::format("Failed to convert surface - SDL error occured %s") % SDL_GetError());
        return false;
    }

    free();
    this->surface = surface;

    return true;
}

void ros::SDLImageBuffer::free() {
    if (!surface) {
        return;
    }
    SDL_FreeSurface(surface);
    surface = ROS_NULL;
}

ros::U32 ros::SDLImageBuffer::getWidth() const {
    if (surface) {
        return static_cast<U32>(surface->w);
    }
    return 0;
}

ros::U32 ros::SDLImageBuffer::getHeight() const {
    if (surface) {
        return static_cast<U32>(surface->h);
    }
    return 0;
}

ros::PixelFormat ros::SDLImageBuffer::getFormat() const {
    if (surface) {
        return PixelFormat_fromSDLFormat(surface->format->format);
    }
    return PixelFormat_Unknown;
}

ros::U32 ros::SDLImageBuffer::getBitsPerPixel() const {
    if (surface) {
        return surface->format->BitsPerPixel;
    }
    return 0;
}

ros::U32 ros::SDLImageBuffer::getPitch() const {
    if (surface) {
        return surface->pitch;
    }
    return 0;
}

ros::U32 ros::SDLImageBuffer::getSize() const {
    if (surface) {
        return static_cast<U32>(surface->pitch * surface->h);
    }
    return 0;
}

bool ros::SDLImageBuffer::clear() {
    if (!surface) {
        return false;
    }

    if (SDL_FillRect(surface, ROS_NULL, SDL_MapRGB(surface->format, 0, 0, 0)) != 0) {
        Logger::report(LogLevel_Error, boost::format("Failed to fill a rectangle on a surface - SDL error occured %s") % SDL_GetError());
        return false;
    }

    return true;
}

void* ros::SDLImageBuffer::lock() {
    if (!surface) {
        return ROS_NULL;
    }

    if (SDL_LockSurface(surface) != 0) {
        Logger::report(LogLevel_Error, boost::format("Failed to lock a surface - SDL error occured %s") % SDL_GetError());
        return ROS_NULL;
    }

    return surface->pixels;
}

const void* ros::SDLImageBuffer::lock() const {
    return const_cast<SDLImageBuffer*>(this)->lock();
}

void ros::SDLImageBuffer::unlock() const {
    if (surface) {
        SDL_UnlockSurface(surface);
    }
}
