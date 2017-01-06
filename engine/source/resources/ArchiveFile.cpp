/*
 * Copyright (c) 2017 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <application/Logger.h>
#include <resources/ArchiveFile.h>

bool ros::ArchiveFile::open(const PropertyTree &config) {
    name = config.data();
    if (name.empty()) {
        Logger::report(LogLevel_Error, boost::format("Archive name is missing"));
        close();
        return false;
    }
    return true;
}

void ros::ArchiveFile::close() {
    name.clear();
}


