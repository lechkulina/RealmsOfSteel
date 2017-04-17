/*
 * Copyright (c) 2017 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <application/Logger.h>
#include <resources/ResourceCache.h>
#include <resources/BufferCacheManager.h>

ros::ResourceCachePtr ros::ResourceCache::instance;

ros::ResourceCachePtr ros::ResourceCache::initInstance(const PropertyTree& config) {
    if (instance) {
        return instance;
    }

    instance.reset(new ResourceCache());
    if (instance && !instance->init(config)) {
        instance.reset();
    }

    return instance;
}

ros::ResourceCache::~ResourceCache() {
    uninit();
}

bool ros::ResourceCache::init(const PropertyTree& config) {
    uninit();

    if (!archiveManager.init(config) || !loaderManager.init(config) || !cacheManager.init(config) ||
        !cacheManager.initCaches(config, caches)) {
        uninit();
        return false;
    }

    return true;
}

void ros::ResourceCache::uninit() {
    caches.clear();
    cacheManager.uninit();
    loaderManager.uninit();
    archiveManager.uninit();
}

ros::ArchiveEntryPtr ros::ResourceCache::findEntry(const std::string& name) const {
    for (BufferCacheList::const_iterator iter = caches.begin(); iter != caches.end(); ++iter) {
        BufferCachePtr cache = *iter;
        ArchiveEntryPtr entry = cache->findEntry(name);
        if (entry) {
            return entry;
        }
    }
    return ArchiveEntryPtr();
}

ros::BufferPtr ros::ResourceCache::acquireBuffer(const std::string& name) {
    for (BufferCacheList::iterator iter = caches.begin(); iter != caches.end(); ++iter) {
        BufferCachePtr cache = *iter;
        BufferPtr buffer = cache->acquireBuffer(name);
        if (buffer) {
            return buffer;
        }
    }
    return BufferPtr();
}

void ros::ResourceCache::releaseBuffer(BufferPtr buffer) {
    for (BufferCacheList::iterator iter = caches.begin(); iter != caches.end(); ++iter) {
        BufferCachePtr cache = *iter;
        cache->releaseBuffer(buffer);
    }
}
