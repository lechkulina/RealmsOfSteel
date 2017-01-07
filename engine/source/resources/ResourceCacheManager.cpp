/*
 * Copyright (c) 2017 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <application/Logger.h>
#include <resources/ResourceCacheManager.h>

ros::ResourceCacheManagerPtr ros::ResourceCacheManager::manager;

ros::ResourceCacheManagerPtr ros::ResourceCacheManager::initInstance(const PropertyTree& config) {
    if (manager) {
        return manager;
    }

    manager.reset(new ResourceCacheManager());
    if (manager && !manager->init(config)) {
        manager.reset();
    }

    return manager;
}

ros::ResourceCacheManager::~ResourceCacheManager() {
    uninit();
}

bool ros::ResourceCacheManager::init(const PropertyTree& config) {
    uninit();

    for (PropertyTree::const_iterator iter = config.begin(); iter != config.end(); ++iter) {
        if (iter->first == "cache" && !initCache(iter->second)) {
            uninit();
            return false;
        }
    }

    return true;
}

void ros::ResourceCacheManager::uninit() {
    caches.clear();
    factory.clear();
}

ros::ResourceCachePtr ros::ResourceCacheManager::initCache(const PropertyTree& config) {
    std::string name = config.data();
    if (name.empty()) {
        Logger::report(LogLevel_Error, boost::format("Cache name is missing"));
        return ResourceCachePtr();
    }

    ResourceCacheMap::iterator iter = caches.find(name);
    if (iter != caches.end()) {
        if (!config.empty()) {
            Logger::report(LogLevel_Warning, boost::format("Cache %s has already been defined - ignoring redefinition") % name);
        }
        return iter->second;
    }

    StringOpt type = config.get_optional<std::string>("type");
    if (!type) {
        Logger::report(LogLevel_Error, boost::format("Missing type property in cache %s") % name);
        return ResourceCachePtr();
    }
    ResourceCachePtr cache(factory.create(type->c_str()));
    if (!cache) {
        Logger::report(LogLevel_Error, boost::format("Unsupported type %s used for cache %s") % type % name);
        return ResourceCachePtr();
    }
    if (!cache->init(config)) {
        return ResourceCachePtr();
    }

    caches[name] = cache;
    return cache;
}

bool ros::ResourceCacheManager::initCaches(const PropertyTree& config, ResourceCacheList& dst) {
    for (PropertyTree::const_iterator iter = config.begin(); iter != config.end(); ++iter) {
        if (iter->first != "cache") {
            continue;
        }
        ResourceCachePtr cache = initCache(iter->second);
        if (!cache) {
            return false;
        }
        dst.push_back(cache);
    }
    return true;
}

