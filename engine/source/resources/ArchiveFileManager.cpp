/*
 * Copyright (c) 2017 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <application/Logger.h>
#include <resources/ArchiveFileManager.h>
#if defined(ROS_USING_ZIP)
    #include "ZIPArchiveFile.h"
#endif

ros::ArchiveFileManager::~ArchiveFileManager() {
    uninit();
}

bool ros::ArchiveFileManager::init(const PropertyTree& config) {
    uninit();

#if defined(ROS_USING_ZIP)
    if (!factory.registerClass<ZIPArchiveFile>(boost::regex(".*zip$"))) {
        Logger::report(LogLevel_Critical, boost::format("Failed to register a zip archive"));
        return false;
    }
#endif

    for (PropertyTree::const_iterator iter = config.begin(); iter != config.end(); ++iter) {
        if (iter->first == "archive" && !openArchive(iter->second)) {
            uninit();
            return false;
        }
    }

    return true;
}

void ros::ArchiveFileManager::uninit() {
    archives.clear();
    factory.clear();
}

ros::ArchiveFilePtr ros::ArchiveFileManager::openArchive(const PropertyTree& config) {
    std::string name = config.data();
    if (name.empty()) {
        Logger::report(LogLevel_Error, boost::format("Archive name is missing"));
        return ArchiveFilePtr();
    }

    ArchiveFileMap::iterator iter = archives.find(name);
    if (iter != archives.end()) {
        if (!config.empty()) {
            Logger::report(LogLevel_Warning, boost::format("Archive %s has already been defined - ignoring redefinition") % name);
        }
        return iter->second;
    }

    StringOpt path = config.get_optional<std::string>("path");
    if (!path) {
        Logger::report(LogLevel_Error, boost::format("Missing path property in archive %s") % name);
        return ArchiveFilePtr();
    }
    ArchiveFilePtr archive(factory.create(path->c_str()));
    if (!archive) {
        Logger::report(LogLevel_Error, boost::format("Unsupported file format %s used for archive %s") % path % name);
        return ArchiveFilePtr();
    }
    if (!archive->open(config)) {
        return ArchiveFilePtr();
    }

    archives[name] = archive;
    return archive;
}

bool ros::ArchiveFileManager::openArchives(const PropertyTree& config, ArchiveFileList& dst) {
    for (PropertyTree::const_iterator iter = config.begin(); iter != config.end(); ++iter) {
        if (iter->first != "archive") {
            continue;
        }
        ArchiveFilePtr archive = openArchive(iter->second);
        if (!archive || !archive->isOpen()) {
            return false;
        }
        dst.push_back(archive);
    }
    return true;
}
