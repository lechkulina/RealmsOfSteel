/*
 * Copyright (c) 2017 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <application/Logger.h>
#include <resources/BufferCacheManager.h>
#include "StaticBufferCache.h"

ros::BufferCacheManagerPtr ros::BufferCacheManager::manager;

ros::BufferCacheManagerPtr ros::BufferCacheManager::initInstance(const PropertyTree& config) {
    if (manager) {
        return manager;
    }

    manager.reset(new BufferCacheManager());
    if (manager && !manager->init(config)) {
        manager.reset();
    }

    return manager;
}

ros::BufferCacheManager::~BufferCacheManager() {
    uninit();
}

bool ros::BufferCacheManager::init(const PropertyTree& config) {
    uninit();

    factory.registerClass<StaticBufferCache>(boost::regex("static"));

    for (PropertyTree::const_iterator iter = config.begin(); iter != config.end(); ++iter) {
        if (iter->first == "cache" && !initCache(iter->second)) {
            uninit();
            return false;
        }
    }

    return true;
}

void ros::BufferCacheManager::uninit() {
    caches.clear();
    factory.clear();
}

ros::BufferCachePtr ros::BufferCacheManager::initCache(const PropertyTree& config) {
    std::string name = config.data();
    if (name.empty()) {
        Logger::report(LogLevel_Error, boost::format("Cache name is missing"));
        return BufferCachePtr();
    }

    BufferCacheMap::iterator iter = caches.find(name);
    if (iter != caches.end()) {
        if (!config.empty()) {
            Logger::report(LogLevel_Warning, boost::format("Cache %s has already been defined - ignoring redefinition") % name);
        }
        return iter->second;
    }

    StringOpt type = config.get_optional<std::string>("type");
    if (!type) {
        Logger::report(LogLevel_Error, boost::format("Missing type property in cache %s") % name);
        return BufferCachePtr();
    }
    BufferCachePtr cache(factory.create(type->c_str()));
    if (!cache) {
        Logger::report(LogLevel_Error, boost::format("Unsupported type %s used for cache %s") % type % name);
        return BufferCachePtr();
    }
    if (!cache->init(config)) {
        return BufferCachePtr();
    }

    caches[name] = cache;
    return cache;
}

bool ros::BufferCacheManager::initCaches(const PropertyTree& config, BufferCacheList& dst) {
    for (PropertyTree::const_iterator iter = config.begin(); iter != config.end(); ++iter) {
        if (iter->first != "cache") {
            continue;
        }
        BufferCachePtr cache = initCache(iter->second);
        if (!cache) {
            return false;
        }
        dst.push_back(cache);
    }
    return true;
}

