/*
 * Copyright (c) 2017 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <application/Logger.h>
#include <resources/ArchiveFile.h>
#ifdef ROS_USING_ZIP
    #include "ZIPArchiveFile.h"
#endif

ros::Factory<ros::ArchiveFile> ros::ArchiveFile::factory;

ros::ArchiveFilePtr ros::ArchiveFile::create(const std::string& classId) {
    if (factory.isEmpty()) {
#ifdef ROS_USING_ZIP
        factory.registerClass<ZIPArchiveFile>(boost::regex(".*zip$"));
#endif
    }
    ArchiveFilePtr archiveFile(factory.create(classId.c_str()));
    return archiveFile;
}

ros::ArchiveEntryPtr ros::ArchiveFile::findEntry(const std::string& name) const {
    ArchiveEntriesMap::const_iterator iter = getEntries().find(name);
    if (iter != getEntries().end()) {
        return iter->second;
    }
    return ArchiveEntryPtr();
}

bool ros::ArchiveFile::hasEntry(const std::string& name) const {
    return findEntry(name).get() != ROS_NULL;
}
