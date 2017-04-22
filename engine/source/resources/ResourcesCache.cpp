/*
 * Copyright (c) 2017 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <resources/ResourcesCache.h>
#include "StaticResourcesCache.h"

ros::Factory<ros::ResourcesCache> ros::ResourcesCache::factory;
ros::ResourcesCachePtr ros::ResourcesCache::instance;

ros::ResourcesCachePtr ros::ResourcesCache::initInstance(const std::string& classId) {
    if (instance) {
        return instance;
    }
    if (factory.isEmpty()) {
        factory.registerClass<StaticResourcesCache>(boost::regex("static-resources-cache"));
    }
    instance.reset(factory.create(classId.c_str()));
    if (!instance) {
        Logger::report(LogLevel_Error, boost::format("Unknown resources cache class ID %s") % classId);
        return ResourcesCachePtr();
    }
    return instance;
}

bool ros::ResourcesCache::addLoader(ResourceLoaderPtr loader) {
    loaders.push_back(loader);
    return true;
}

ros::ResourceLoaderPtr ros::ResourcesCache::findLoader(const std::string& resourceName) const {
    for (ResourceLoadersList::const_iterator i = loaders.begin(); i != loaders.end(); ++i) {
        ResourceLoaderPtr loader = *i;
        if (loader->isLoadable(resourceName)) {
            return loader;
        }
    }
    return ResourceLoaderPtr();
}
