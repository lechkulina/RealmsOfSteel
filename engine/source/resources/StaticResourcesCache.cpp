/*
 * Copyright (c) 2017 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <application/Logger.h>
#include <resources/FileSystem.h>
#include "StaticResourcesCache.h"

ros::ResourcePtr ros::StaticResourcesCache::readResource(const std::string& resourceName) {
    ResourcesMap::const_iterator found = resources.find(resourceName);
    if (found != resources.end()) {
        Logger::report(LogLevel_Trace, boost::format("Resource %s found in cache") % resourceName);
        return found->second;
    }

    ResourceLoaderPtr loader = findLoader(resourceName);
    if (!loader) {
        Logger::report(LogLevel_Warning, boost::format("Failed to find a suitable loader for resource %s") % resourceName);
        return ResourcePtr();
    }
    ResourcePtr resource = loader->load(resourceName);
    if (!resource) {
        Logger::report(LogLevel_Warning, boost::format("Failed to load resource %s") % resourceName);
        return ResourcePtr();
    }

    if (capacity) {
        S32 exceededSize = resource->getSize() + computeUsedSize() - (*capacity);
        if (exceededSize > 0) {
            Logger::report(LogLevel_Warning, boost::format("Resource %s uses %d bytes and exceeds cache capacity by %d bytes - deleting it")
                                % resourceName % resource->getSize() % exceededSize);
            return ResourcePtr();
        }
    }

    Logger::report(LogLevel_Trace, boost::format("Resource %s loaded into in cache") % resourceName);
    resources[resourceName] = resource;

    return resource;
}

bool ros::StaticResourcesCache::hasResource(const std::string& resourceName) const {
    return resources.find(resourceName) != resources.end();
}

ros::U32 ros::StaticResourcesCache::computeUsedSize() const {
    U32 size = 0;
    for (ResourcesMap::const_iterator i = resources.begin(); i != resources.end(); ++i) {
        ResourcePtr resource = i->second;
        size += resource->getSize();
    }
    return size;
}
