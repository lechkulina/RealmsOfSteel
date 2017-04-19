/*
 * Copyright (c) 2017 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <core/Factory.h>
#include <application/Logger.h>
#include <resources/ArchiveFile.h>
#ifdef ROS_USING_ZIP
    #include "ZIPArchiveFile.h"
#endif

static ros::Factory<ros::ArchiveFile> factory;

ros::ArchiveFilePtr ros::ArchiveFile::create(const std::string& classId) {
    if (factory.isEmpty()) {
#ifdef ROS_USING_ZIP
        factory.registerClass<ZIPArchiveFile>(boost::regex(".*zip$"));
#endif
    }
    ArchiveFilePtr archiveFile(factory.create(classId.c_str()));
    return archiveFile;
}
