/*
 * Copyright (c) 2017 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <application/Logger.h>
#include <resources/FileSystem.h>
#include "StaticResourcesCache.h"

ros::ResourcePtr ros::StaticResourcesCache::readResource(const std::string& name) {
    ResourcesMap::const_iterator found = resources.find(name);
    if (found != resources.end()) {
        Logger::report(LogLevel_Trace, boost::format("Resource %s found in cache") % name);
        return found->second;
    }

    ResourceLoaderPtr loader = findLoaderForResource(name);
    if (!loader) {
        Logger::report(LogLevel_Warning, boost::format("Failed to find a loader for resource %s") % name);
        return ResourcePtr();
    }
    ResourcePtr resource = loader->loadResource(name);
    if (!resource) {
        Logger::report(LogLevel_Warning, boost::format("Failed to load resource %s") % name);
        return ResourcePtr();
    }

    if (capacity) {
        const S32 exceededSize = resource->getSize() + computeUsedSize() - (*capacity);
        if (exceededSize > 0) {
            Logger::report(LogLevel_Warning, boost::format("Resource %s uses %d bytes and exceeds cache capacity by %d bytes - deleting it")
                                % name % resource->getSize() % exceededSize);
            return ResourcePtr();
        }
    }

    Logger::report(LogLevel_Trace, boost::format("Resource %s loaded into in cache") % name);
    resources[name] = resource;

    return resource;
}

bool ros::StaticResourcesCache::hasResource(const std::string& name) const {
    return resources.find(name) != resources.end();
}

ros::U32 ros::StaticResourcesCache::computeUsedSize() const {
    U32 size = 0;
    for (ResourcesMap::const_iterator iter = resources.begin(); iter != resources.end(); ++iter) {
        ResourcePtr resource = iter->second;
        size += resource->getSize();
    }
    return size;
}
