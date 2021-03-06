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
#ifdef ROS_USING_ASSIMP
    #include "AssimpSceneLoader.h"
#endif

static ros::Factory<ros::ResourceLoader> factory;

ros::ResourceLoaderPtr ros::ResourceLoader::create(const std::string& classId) {
    if (factory.isEmpty()) {
        factory.registerClass<RawBufferLoader>(boost::regex("raw-buffer"));
#if defined(ROS_USING_SDL) && defined(ROS_USING_SDL_IMAGE)
        factory.registerClass<SDLImageLoader>(boost::regex("sdl-image"));
#endif
#ifdef ROS_USING_ASSIMP
        factory.registerClass<AssimpSceneLoader>(boost::regex("assimp-scene"));
#endif
    }
    ResourceLoaderPtr instance(factory.create(classId));
    return instance;
}
