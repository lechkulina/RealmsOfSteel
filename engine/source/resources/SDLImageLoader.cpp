/*
 * Copyright (c) 2017 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <boost/regex.hpp>
#include <SDL_image.h>
#include <application/Logger.h>
#include <resources/FileSystem.h>
#include "../graphics/SDLImage.h"
#include "SDLImageLoader.h"

bool ros::SDLImageLoader::isLoadable(const std::string& resourceName) const {
    // TODO use SDL API to check if a resource can be loaded
    boost::regex loadableRegex(".*tga|.*bmp|.*pbm|.*pgm|.*ppm|.*xpm|.*xcf|.*gif|.*lbm|.*iff|.*jpg|.*tif|.*png");
    return boost::regex_match(resourceName, loadableRegex);
}

ros::ResourcePtr ros::SDLImageLoader::load(const std::string& resourceName) {
    RawBufferPtr buffer = FileSystem::getInstance()->readFile(resourceName);
    if (!buffer) {
        return SDLImagePtr();
    }
    SDL_RWops* stream = SDL_RWFromConstMem(buffer->at<const void>(), buffer->getSize());
    if (!stream) {
        Logger::report(LogLevel_Error, boost::format("Failed to create SDL RWops stream for resource %s - SDL error occured %s")
                            % resourceName % SDL_GetError());
        return SDLImagePtr();
    }

    SDL_Surface* surface = IMG_Load_RW(stream, 0);
    if (!surface) {
        Logger::report(LogLevel_Error, boost::format("Failed to load SDL surface for resource %s - SDLImage error occured %s")
                            % resourceName % IMG_GetError());
        SDL_RWclose(stream);
        return SDLImagePtr();
    }

    SDLImagePtr image = boost::make_shared<SDLImage>(surface);
    if (!image || image->isNull()) {
        Logger::report(LogLevel_Error, boost::format("Failed to create resource for %s") % resourceName);
        SDL_FreeSurface(surface);
        SDL_RWclose(stream);
        return SDLImagePtr();
    }

    // image buffer is now the owner of the surface and it will free it when necessary
    SDL_RWclose(stream);
    return image;
}

