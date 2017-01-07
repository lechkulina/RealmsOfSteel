/*
 * Copyright (c) 2017 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <application/Logger.h>
#include <resources/ArchiveFileManager.h>
#include <resources/ResourceLoaderManager.h>
#include <resources/ResourceCache.h>

bool ros::ResourceCache::init(const PropertyTree &config) {
    name = config.data();
    if (name.empty()) {
        Logger::report(LogLevel_Error, boost::format("Cache name is missing"));
        uninit();
        return false;
    }
    return true;
}

void ros::ResourceCache::uninit() {
    name.clear();
    archives.clear();
    loaders.clear();
}

bool ros::ResourceCache::initArchives(const PropertyTree& config) {
    for (PropertyTree::const_iterator iter = config.begin(); iter != config.end(); ++iter) {
        if (iter->first != "archive") {
            continue;
        }
        ArchiveFilePtr archive = ArchiveFileManager::getInstance()->openArchive(iter->second);
        if (!archive || !archive->isOpen()) {
            uninit();
            return false;
        }
        archives.push_back(archive);
    }
    return true;
}

bool ros::ResourceCache::initLoaders(const PropertyTree& config) {
    for (PropertyTree::const_iterator iter = config.begin(); iter != config.end(); ++iter) {
        if (iter->first != "loader") {
            continue;
        }
        ResourceLoaderPtr loader = ResourceLoaderManager::getInstance()->initLoader(iter->second);
        if (!loader) {
            uninit();
            return false;
        }
        loaders.push_back(loader);
    }
    return true;
}

ros::BufferPtr ros::ResourceCache::loadBuffer(const std::string& name) {
    ArchiveEntryPtr entry = findEntry(name);
    if (!entry) {
        Logger::report(LogLevel_Error, boost::format("Failed to find archive entry for resource %s") % name);
        return BufferPtr();
    }

    RawBufferPtr src = entry->decompress();
    if (!src || src->isNull()) {
        return BufferPtr();
    }

    ResourceLoaderPtr loader = findLoader(name);
    if (!loader) {
        Logger::report(LogLevel_Error, boost::format("Failed to find loader that could handle resource %s") % name);
        return BufferPtr();
    }

    return loader->loadBuffer(src);
}

ros::ArchiveEntryPtr ros::ResourceCache::findEntry(const std::string& name) {
    for (ArchiveFileList::iterator iter = archives.begin(); iter != archives.end(); ++iter) {
        ArchiveEntryMap& entries = (*iter)->getEntries();
        ArchiveEntryMap::iterator found = entries.find(name);
        if (found != entries.end()) {
            return found->second;
        }
    }
    return ArchiveEntryPtr();
}

ros::ResourceLoaderPtr ros::ResourceCache::findLoader(const std::string& name) {
    for (ResourceLoaderList::iterator iter = loaders.begin(); iter != loaders.end(); ++iter) {
        if ((*iter)->isLoadable(name)) {
            return *iter;
        }
    }
    return ResourceLoaderPtr();
}

