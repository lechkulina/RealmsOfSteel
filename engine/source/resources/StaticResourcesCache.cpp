/*
 * Copyright (c) 2017 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <application/Logger.h>
#include <resources/FileSystem.h>
#include "StaticResourcesCache.h"

ros::StaticResourcesCache::StaticResourcesCache()
    : usedSize(0) {
}

bool ros::StaticResourcesCache::hasResource(const std::string& name) const {
    return resources.find(name) != resources.end();
}

void ros::StaticResourcesCache::setCapacity(U32Opt capacity) {
    this->capacity = capacity;
}

void ros::StaticResourcesCache::dropResource(const std::string& name) {
    resources.erase(name);
}

ros::ResourcePtr ros::StaticResourcesCache::loadResource(const std::string& name) {
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

    const U32 resourceSize = resource->getSize();
    if (capacity) {
        const S32 exceededSize = resourceSize + usedSize - (*capacity);
        if (exceededSize > 0) {
            Logger::report(LogLevel_Warning, boost::format("Resource %s uses %d bytes and exceeds cache capacity by %d bytes - deleting it")
                                % name % resourceSize % exceededSize);
            return ResourcePtr();
        }
    }

    Logger::report(LogLevel_Trace, boost::format("Resource %s (uses %d bytes) loaded into in cache") % name % resourceSize);
    resource->setCache(ResourcesCache::getInstance());
    usedSize += resourceSize;
    resources[name] = resource;

    return resource;
}

void ros::StaticResourcesCache::onResourceSizeChanged(S32 delta) {
    usedSize += delta;
    Logger::report(LogLevel_Trace, boost::format("Resources cache used size %d was changed by %d bytes") % usedSize % delta);
}
