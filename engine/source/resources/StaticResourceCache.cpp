/*
 * Copyright (c) 2017 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <application/Logger.h>
#include "StaticResourceCache.h"

ros::StaticResourceCache::~StaticResourceCache() {
    uninit();
}

bool ros::StaticResourceCache::init(const PropertyTree& config) {
    if (!ResourceCache::init(config)) {
        return false;
    }

    capacity = config.get_optional<U32>("capacity");
    if (!capacity) {
        Logger::report(LogLevel_Debug, boost::format("Cache %s will use infinite capacity") % getName());
    }

    return true;
}

void ros::StaticResourceCache::uninit() {
    capacity.reset();
    buffers.clear();
    ResourceCache::uninit();
}

ros::BufferPtr ros::StaticResourceCache::acquireBuffer(const std::string& name) {
    BufferMap::iterator found = buffers.find(name);
    if (found != buffers.end()) {
        return found->second;
    }

    BufferPtr buffer = loadBuffer(name);
    if (!buffer) {
        return BufferPtr();
    }

    if (capacity) {
        S32 exceededSize = buffer->getSize() + computeUsedSize() - (*capacity);
        if (exceededSize > 0) {
            Logger::report(LogLevel_Warning, boost::format("Resource %s uses %d bytes and exceeds cache %s capacity by %d bytes - deleting it")
                                % name % buffer->getSize() % getName() % exceededSize);
            return BufferPtr();
        }
    }

    buffers[name] = buffer;
    return buffer;

}

ros::U32 ros::StaticResourceCache::computeUsedSize() const {
    U32 size = 0;
    for (BufferMap::const_iterator iter = buffers.begin(); iter != buffers.end(); ++iter) {
        size += iter->second->getSize();
    }
    return size;
}
