/*
 * Copyright (c) 2017 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <SDL_image.h>
#include <application/Logger.h>
#include "../graphics/SDLImageBuffer.h"
#include "SDLImageBufferLoader.h"

ros::SDLImageBufferLoader::~SDLImageBufferLoader() {
    uninit();
}

bool ros::SDLImageBufferLoader::init(const PropertyTree &config) {
    uninit();
    if (!BufferLoader::init(config)) {
        uninit();
        return false;
    }

    StringOpt loadableOpt = config.get_optional<std::string>("loadable");
    if (!loadableOpt) {
        Logger::report(LogLevel_Error, boost::format("Missing loadable property in loader %s") % getName());
        uninit();
        return false;
    }
    loadable.assign(*loadableOpt);

    return true;
}

void ros::SDLImageBufferLoader::uninit() {
    loadable.assign("");
    BufferLoader::uninit();
}

bool ros::SDLImageBufferLoader::isLoadable(const std::string& name) const {
    if (loadable.empty()) {
        return false;
    }
    return boost::regex_match(name, loadable);
}

ros::BufferPtr ros::SDLImageBufferLoader::loadBuffer(RawBufferPtr src) {
    SDL_RWops* stream = SDL_RWFromConstMem(src->at<const void>(), src->getSize());
    if (!stream) {
        Logger::report(LogLevel_Error, boost::format("Failed to create stream for loader %s - SDL error occured %s")
                            % getName() % SDL_GetError());
        return SDLImageBufferPtr();
    }

    SDL_Surface* surface = IMG_Load_RW(stream, 0);
    if (!surface) {
        Logger::report(LogLevel_Error, boost::format("Failed to load resource from loader %s - SDLImage error occured %s")
                            % getName() % IMG_GetError());
        SDL_RWclose(stream);
        return SDLImageBufferPtr();
    }

    SDLImageBufferPtr dst = boost::make_shared<SDLImageBuffer>(surface);
    if (!dst || dst->isNull()) {
        Logger::report(LogLevel_Error, boost::format("Failed to create destination buffer for loader %s") % getName());
        SDL_FreeSurface(surface);
        SDL_RWclose(stream);
        return SDLImageBufferPtr();
    }

    // image buffer is now the owner of the surface and it will free it when necessary
    SDL_RWclose(stream);
    return dst;
}

