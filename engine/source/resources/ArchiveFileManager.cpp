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

ros::ArchiveFileManagerPtr ros::ArchiveFileManager::manager;

ros::ArchiveFileManagerPtr ros::ArchiveFileManager::initInstance(const PropertyTree& config) {
    if (manager) {
        return manager;
    }

    manager.reset(new ArchiveFileManager());
    if (manager && !manager->init(config)) {
        manager.reset();
    }

    return manager;
}

ros::ArchiveFileManager::~ArchiveFileManager() {
    uninit();
}

bool ros::ArchiveFileManager::init(const PropertyTree& config) {
    uninit();

#if defined(ROS_USING_ZIP)
    factory.registerClass<ZIPArchiveFile>(boost::regex(".*zip$"));
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

    StringOpt path = config.get_optional<std::string>("file-path");
    if (!path) {
        Logger::report(LogLevel_Error, boost::format("Missing file path property in archive %s") % name);
        return ArchiveFilePtr();
    }
    ArchiveFilePtr archive(factory.create(path->c_str()));
    if (!archive) {
        Logger::report(LogLevel_Error, boost::format("Unsupported file format %s used for archive %s") % path % name);
        return ArchiveFilePtr();
    }
    if (!archive->open(path->c_str())) {
        return ArchiveFilePtr();
    }

    archives[name] = archive;
    return archive;
}
