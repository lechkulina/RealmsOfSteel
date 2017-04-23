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

ros::ResourcesCachePtr ros::ResourcesCache::initInstance(const pt::ptree& config) {
    if (instance) {
        return instance;
    }

    if (factory.isEmpty()) {
        factory.registerClass<StaticResourcesCache>(boost::regex("static"));
    }

    const std::string classId = config.data();
    instance.reset(factory.create(classId));
    if (!instance) {
        Logger::report(LogLevel_Error, boost::format("Unknown resources cache class ID %s") % classId);
        return ResourcesCachePtr();
    }
    instance->setCapacity(config.get_optional<U32>("capacity"));
    for (pt::ptree::const_iterator iter = config.begin(); iter != config.end(); ++iter) {
        if (iter->first == "loader") {
            const pt::ptree& loaderConfig = iter->second;
            const std::string& loaderClassId = loaderConfig.data();
            ResourceLoaderPtr loader = ros::ResourceLoader::create(loaderClassId);
            if (!loader) {
                Logger::report(LogLevel_Error, boost::format("Unknown resource loader class ID %s") % loaderClassId);
                continue;
            }
            Logger::report(LogLevel_Trace, boost::format("Loader with class ID %s added into in cache") % loaderClassId);
            instance->addLoader(loader);
        }
    }

    Logger::report(LogLevel_Trace, boost::format("Resources cache with class ID %s initialized") % classId);
    return instance;
}

void ros::ResourcesCache::addLoader(ResourceLoaderPtr loader) {
    loaders.push_back(loader);
}

ros::ResourceLoaderPtr ros::ResourcesCache::findLoaderForResource(const std::string& name) const {
    for (ResourceLoadersList::const_iterator iter = loaders.begin(); iter != loaders.end(); ++iter) {
        ResourceLoaderPtr loader = *iter;
        if (loader->isLoadable(name)) {
            return loader;
        }
    }
    return ResourceLoaderPtr();
}
