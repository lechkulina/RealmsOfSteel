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

ros::ArchiveEntryPtr ros::ArchiveFile::findEntry(const std::string& entryName) const {
    ArchiveEntriesMap::const_iterator foundEntry = getEntries().find(entryName);
    if (foundEntry != getEntries().end()) {
        return foundEntry->second;
    }
    return ArchiveEntryPtr();
}

bool ros::ArchiveFile::hasEntry(const std::string& entryName) const {
    return findEntry(entryName).get() != ROS_NULL;
}
