/*
 * Copyright (c) 2017 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <core/Factory.h>
#include <application/Logger.h>
#include <resources/FileSystem.h>
#include "ArchiveFileSystem.h"

ros::Factory<ros::FileSystem> ros::FileSystem::factory;
ros::FileSystemPtr ros::FileSystem::fileSystem;

ros::FileSystemPtr ros::FileSystem::initInstance(const std::string& classId) {
    if (fileSystem) {
        return fileSystem;
    }
    if (factory.isEmpty()) {
        factory.registerClass<ArchiveFileSystem>(boost::regex("archive-file-system"));
    }
    fileSystem.reset(factory.create(classId.c_str()));
    if (!fileSystem) {
        Logger::report(LogLevel_Error, boost::format("Unknown file system class ID %s") % classId);
        return FileSystemPtr();
    }
    return fileSystem;
}
