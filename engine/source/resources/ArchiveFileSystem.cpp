/*
 * Copyright (c) 2017 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <boost/filesystem.hpp>
#include <application/Logger.h>
#include "ArchiveFileSystem.h"

bool ros::ArchiveFileSystem::addArchive(ArchiveFilePtr archive) {
    const fs::path& path = archive->getPath();
    for (ArchiveFilesList::const_iterator iter = archives.begin(); iter != archives.end(); ++iter) {
        if ((*iter)->getPath() == path) {
            ROS_WARNING(boost::format("Archive %s has already been added into file system - ignoring it") % path);
            return false;
        }
    }
    ROS_TRACE(boost::format("Archive %s added into file system") % path);
    archives.push_back(archive);
    return true;
}

bool ros::ArchiveFileSystem::openArchive(const fs::path& path) {
    ArchiveFilePtr archive = ArchiveFile::create(path.string());
    if (!archive) {
        ROS_ERROR(boost::format("Failed to create archive %s for file system") % path);
        return false;
    }
    if (!archive->open(path)) {
        ROS_ERROR(boost::format("Failed to open archive %s for file system") % path);
        return false;
    }
    return addArchive(archive);
}

bool ros::ArchiveFileSystem::setRoot(const fs::path& root) {
    this->root.clear();
    archives.clear();

    sys::error_code error;
    const bool isDirectory = fs::is_directory(root, error);
    if (error) {
        ROS_ERROR(boost::format("Failed to check type for root %s - system error occured %s") % root % error.message());
        return false;
    }
    if (!isDirectory) {
        ROS_ERROR(boost::format("Provided root %s does not point to a directory") % root);
        return false;
    }

    fs::directory_iterator iter(root, error);
    if (error) {
        ROS_ERROR(boost::format("Failed to open root %s - system error occured %s") % root % error.message());
        return false;
    }
    while (iter != fs::directory_iterator()) {
        const fs::directory_entry entry = *iter;
        openArchive(entry.path());
        ++iter;
    }

    ROS_DEBUG(boost::format("Root %s set with %d opened archives") % root % archives.size());
    return archives.size() > 0;
}

ros::RawBufferPtr ros::ArchiveFileSystem::readFile(const std::string& name) const {
    for (ArchiveFilesList::const_iterator iter = archives.begin(); iter != archives.end(); ++iter) {
        ArchiveFilePtr archive = *iter;
        ArchiveEntryPtr entry = archive->findEntry(name);
        if (entry) {
            ROS_TRACE(boost::format("Found entry for file %s in file system") % name);
            return entry->decompress();
        }
    }
    ROS_WARNING(boost::format("Failed to find entry for file %s in file system") % name);
    return RawBufferPtr();
}

bool ros::ArchiveFileSystem::hasFile(const std::string& name) const {
    for (ArchiveFilesList::const_iterator iter = archives.begin(); iter != archives.end(); ++iter) {
        ArchiveFilePtr archive = *iter;
        if (archive->hasEntry(name)) {
            return true;
        }
    }
    return false;
}
