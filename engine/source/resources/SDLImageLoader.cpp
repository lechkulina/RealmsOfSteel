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

bool ros::SDLImageLoader::isLoadable(const std::string& name) const {
    // TODO use SDL API to check if a resource can be loaded
    boost::regex regex(".*tga|.*bmp|.*pbm|.*pgm|.*ppm|.*xpm|.*xcf|.*gif|.*lbm|.*iff|.*jpg|.*tif|.*png");
    return boost::regex_match(name, regex);
}

ros::ResourcePtr ros::SDLImageLoader::loadResource(const std::string& name) {
    RawBufferPtr buffer = FileSystem::getInstance()->readFile(name);
    if (!buffer) {
        return SDLImagePtr();
    }
    SDL_RWops* stream = SDL_RWFromConstMem(buffer->at<const void>(), buffer->getSize());
    if (!stream) {
        ROS_ERROR(boost::format("Failed to create SDL RWops stream for resource %s - SDL error occured %s") % name % SDL_GetError());
        return SDLImagePtr();
    }

    SDL_Surface* surface = IMG_Load_RW(stream, 0);
    if (!surface) {
        ROS_ERROR(boost::format("Failed to load SDL surface for resource %s - SDLImage error occured %s")  % name % IMG_GetError());
        SDL_RWclose(stream);
        return SDLImagePtr();
    }

    SDLImagePtr image = boost::make_shared<SDLImage>(surface);
    if (!image || image->isNull()) {
        ROS_ERROR(boost::format("Failed to create image resource for %s") % name);
        SDL_FreeSurface(surface);
        SDL_RWclose(stream);
        return SDLImagePtr();
    }

    // image buffer is now the owner of the surface and it will free it when necessary
    SDL_RWclose(stream);
    return image;
}

