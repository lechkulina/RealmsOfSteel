/*
 * Copyright (c) 2017 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <application/Logger.h>
#include "ArchiveFileSystem.h"

ros::ArchiveFileSystem::ArchiveFileSystem() {
    // TODO do not hardcode this
    openArchiveFile("common.zip");
}

bool ros::ArchiveFileSystem::addArchiveFile(ArchiveFilePtr archiveFile) {
    for (ArchiveFileList::const_iterator i = archiveFiles.begin(); i != archiveFiles.end(); ++i) {
        ArchiveFilePtr archiveFile = *i;
        if (archiveFile->getPath() == archiveFile->getPath()) {
            return false;
        }
    }
    archiveFiles.push_back(archiveFile);
    return true;
}

bool ros::ArchiveFileSystem::openArchiveFile(const std::string& path) {
    ArchiveFilePtr archiveFile = ArchiveFile::create(path);
    if (!archiveFile || !archiveFile->open(path)) {
        Logger::report(LogLevel_Error, boost::format("Failed to add archive file %s into file system") % path);
        return false;
    }
    return addArchiveFile(archiveFile);
}

ros::RawBufferPtr ros::ArchiveFileSystem::readFile(const std::string& fileName) const {
    for (ArchiveFileList::const_iterator i = archiveFiles.begin(); i != archiveFiles.end(); ++i) {
        ArchiveFilePtr archiveFile = *i;
        ArchiveEntryPtr archiveEntry = archiveFile->findEntry(fileName);
        if (archiveEntry) {
            return archiveEntry->decompress();
        }
    }
    return RawBufferPtr();
}

bool ros::ArchiveFileSystem::hasFile(const std::string& fileName) const {
    for (ArchiveFileList::const_iterator i = archiveFiles.begin(); i != archiveFiles.end(); ++i) {
        ArchiveFilePtr archiveFile = *i;
        if (archiveFile->hasEntry(fileName)) {
            return true;
        }
    }
    return false;
}
