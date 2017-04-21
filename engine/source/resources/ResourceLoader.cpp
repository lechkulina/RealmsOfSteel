/*
 * Copyright (c) 2017 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <application/Logger.h>
#include <resources/ResourceLoader.h>
#include "RawBufferLoader.h"
#if defined(ROS_USING_SDL) && defined(ROS_USING_SDL_IMAGE)
    #include "SDLImageLoader.h"
#endif

static ros::Factory<ros::ResourceLoader> factory;

ros::ResourceLoaderPtr ros::ResourceLoader::create(const std::string& classId) {
    if (factory.isEmpty()) {
        factory.registerClass<RawBufferLoader>(boost::regex("raw-buffer-loader"));
#if defined(ROS_USING_SDL) && defined(ROS_USING_SDL_IMAGE)
        factory.registerClass<SDLImageLoader>(boost::regex("sdl-image-loader"));
#endif
    }
    ResourceLoaderPtr resourceLoader(factory.create(classId.c_str()));
    return resourceLoader;
}
