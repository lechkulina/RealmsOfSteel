/*
 * Copyright (c) 2017 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <resources/ResourcesCache.h>
#include <resources/Resource.h>

ros::Resource::Resource()
    : size(0) {
}

void ros::Resource::setCache(ResourcesCacheWeakPtr cache) {
    this->cache = cache;
}

void ros::Resource::setSize(U32 size) {
    ResourcesCachePtr locked = cache.lock();  // not all resource have to be created by the resources cache
    const S32 delta = size - this->size;
    if (locked && delta != 0) {
        locked->onResourceSizeChanged(delta);
    }
    this->size = size;
}
