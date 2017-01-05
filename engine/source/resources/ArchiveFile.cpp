/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <boost/regex.hpp>
#include <application/Logger.h>
#include <resources/ArchiveFile.h>
#if defined(ROS_USING_ZIP)
    #include "ZIPArchiveFile.h"
#endif

ros::ArchiveFileFactory ros::ArchiveFile::factory;

ros::ArchiveFilePtr ros::ArchiveFile::create(const char* path) {
    if (factory.isEmpty()) {
#if defined(ROS_USING_ZIP)
        factory.registerClass<ZIPArchiveFile>(boost::regex(".*zip$"));
#endif
    }

    ArchiveFilePtr file(factory.create(path));
    if (!file) {
        Logger::report(LogLevel_Error, boost::format("Failed to find support for archive file %s") % path);
        return ArchiveFilePtr();
    }
    if (!file->open(path)) {
        return ArchiveFilePtr();
    }

    return file;
}
