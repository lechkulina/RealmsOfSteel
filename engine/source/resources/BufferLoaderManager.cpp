/*
 * Copyright (c) 2017 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <application/Logger.h>
#include <resources/BufferLoaderManager.h>
#include "RawBufferLoader.h"
#if defined(ROS_USING_SDL) && defined(ROS_USING_SDL_IMAGE)
    #include "SDLImageBufferLoader.h"
#endif

ros::BufferLoaderManager::~BufferLoaderManager() {
    uninit();
}

bool ros::BufferLoaderManager::init(const PropertyTree& config) {
    uninit();

    if (!factory.registerClass<RawBufferLoader>(boost::regex("raw"))) {
        Logger::report(LogLevel_Critical, boost::format("Failed to register a raw loader"));
        return false;
    }
#if defined(ROS_USING_SDL) && defined(ROS_USING_SDL_IMAGE)
    if (!factory.registerClass<SDLImageBufferLoader>(boost::regex("sdl-image"))) {
        Logger::report(LogLevel_Critical, boost::format("Failed to register a sdl image loader"));
        return false;
    }
#endif

    for (PropertyTree::const_iterator iter = config.begin(); iter != config.end(); ++iter) {
        if (iter->first == "loader" && !initLoader(iter->second)) {
            uninit();
            return false;
        }
    }

    return true;
}

void ros::BufferLoaderManager::uninit() {
    loaders.clear();
    factory.clear();
}

ros::BufferLoaderPtr ros::BufferLoaderManager::initLoader(const PropertyTree& config) {
    std::string name = config.data();
    if (name.empty()) {
        Logger::report(LogLevel_Error, boost::format("Loader name is missing"));
        return BufferLoaderPtr();
    }

    BufferLoaderMap::iterator iter = loaders.find(name);
    if (iter != loaders.end()) {
        if (!config.empty()) {
            Logger::report(LogLevel_Warning, boost::format("Loader %s has already been defined - ignoring redefinition") % name);
        }
        return iter->second;
    }

    StringOpt type = config.get_optional<std::string>("type");
    if (!type) {
        Logger::report(LogLevel_Error, boost::format("Missing type property in loader %s") % name);
        return BufferLoaderPtr();
    }
    BufferLoaderPtr loader(factory.create(type->c_str()));
    if (!loader) {
        Logger::report(LogLevel_Error, boost::format("Unsupported type %s used for loader %s") % type % name);
        return BufferLoaderPtr();
    }
    if (!loader->init(config)) {
        return BufferLoaderPtr();
    }

    loaders[name] = loader;
    return loader;
}

bool ros::BufferLoaderManager::initLoaders(const PropertyTree& config, BufferLoaderList& dst) {
    for (PropertyTree::const_iterator iter = config.begin(); iter != config.end(); ++iter) {
        if (iter->first != "loader") {
            continue;
        }
        BufferLoaderPtr loader = initLoader(iter->second);
        if (!loader) {
            return false;
        }
        dst.push_back(loader);
    }
    return true;
}
