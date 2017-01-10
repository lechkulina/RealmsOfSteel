/*
 * Copyright (c) 2017 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <application/Logger.h>
#include <resources/ResourceCache.h>
#include <resources/BufferCache.h>

bool ros::BufferCache::init(const PropertyTree &config) {
    name = config.data();
    if (name.empty()) {
        Logger::report(LogLevel_Error, boost::format("Cache name is missing"));
        uninit();
        return false;
    }

    if (!ResourceCache::getInstance()->getArchiveManager().openArchives(config, archives) ||
        !ResourceCache::getInstance()->getLoaderManager().initLoaders(config, loaders)) {
        uninit();
        return false;
    }

    return true;
}

void ros::BufferCache::uninit() {
    name.clear();
    archives.clear();
    loaders.clear();
}

ros::BufferPtr ros::BufferCache::loadBuffer(const std::string& name) {
    ArchiveEntryPtr entry = findEntry(name);
    if (!entry) {
        Logger::report(LogLevel_Error, boost::format("Failed to find entry for resource %s") % name);
        return BufferPtr();
    }

    RawBufferPtr src = entry->decompress();
    if (!src || src->isNull()) {
        return BufferPtr();
    }

    BufferLoaderPtr loader = findLoader(name);
    if (!loader) {
        Logger::report(LogLevel_Error, boost::format("Failed to find loader that could handle resource %s") % name);
        return BufferPtr();
    }

    return loader->loadBuffer(src);
}

ros::ArchiveEntryPtr ros::BufferCache::findEntry(const std::string& name) {
    for (ArchiveFileList::iterator iter = archives.begin(); iter != archives.end(); ++iter) {
        ArchiveEntryMap& entries = (*iter)->getEntries();
        ArchiveEntryMap::iterator found = entries.find(name);
        if (found != entries.end()) {
            return found->second;
        }
    }
    return ArchiveEntryPtr();
}

ros::BufferLoaderPtr ros::BufferCache::findLoader(const std::string& name) {
    for (BufferLoaderList::iterator iter = loaders.begin(); iter != loaders.end(); ++iter) {
        if ((*iter)->isLoadable(name)) {
            return *iter;
        }
    }
    return BufferLoaderPtr();
}

