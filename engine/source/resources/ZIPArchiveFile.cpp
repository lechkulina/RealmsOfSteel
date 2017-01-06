/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <core/RawBuffer.h>
#include <application/Logger.h>
#include "ZIPArchiveEntry.h"
#include "ZIPArchiveFile.h"

ros::ZIPArchiveFile::ZIPArchiveFile(const char* path) {
    open(path);
}

ros::ZIPArchiveFile::~ZIPArchiveFile() {
    close();
}

bool ros::ZIPArchiveFile::open(const char* path) {
    close();
    if (!initFile(path) || !readEntries()) {
        close();
        return false;
    }
    return true;
}

void ros::ZIPArchiveFile::close() {
    entries.clear();
    path.clear();
    file.reset();
}

bool ros::ZIPArchiveFile::isOpen() const {
    return file && file->isOpen();
}

bool ros::ZIPArchiveFile::initFile(const char* path) {
    file = boost::make_shared<RawFile>(path, FileType_Binary, FileOpenMode_Read);
    if (!file || !file->isOpen()) {
        Logger::report(LogLevel_Error, boost::format("Failed to open file %s for reading") % path);
        return false;
    }
    this->path = path;
    return true;
}

bool ros::ZIPArchiveFile::readEntries() {
    ZIPDirectoryRecord record;
    memset(&record, 0, sizeof(record));

    U32 position = -sizeof(U32);
    while(true) {
        if (!file->seek(position, FileOrigin_End) || !file->readLittle(record.signature)) {
            Logger::report(LogLevel_Error, boost::format("Failed to find directory record signature 0x%x in file %s")
                                % ZIPDirectoryRecord_Signature % file->getPath());
            return false;
        }
        if (record.signature == ZIPDirectoryRecord_Signature) {
            break;
        }
        --position;
    }

    if (!file->readLittle(record.diskNumber) ||
        !file->readLittle(record.diskStart) ||
        !file->readLittle(record.entriesCount) ||
        !file->readLittle(record.entriesTotalCount) ||
        !file->readLittle(record.directorySize) ||
        !file->readLittle(record.directoryOffset) ||
        !file->readLittle(record.commentLength)) {
        Logger::report(LogLevel_Error, boost::format("Failed to read directory record from file %s") % file->getPath());
        return false;
    }

    RawBuffer buffer(record.directorySize);
    if (buffer.isNull() || !file->seek(record.directoryOffset, FileOrigin_Begin) || !file->read(buffer.at<void>(), buffer.getSize())) {
        Logger::report(LogLevel_Error, boost::format("Failed to read directory headers from file %s") % file->getPath());
        return false;
    }

    for (U16 i=0; i < record.entriesCount; ++i) {
        ZIPDirectoryHeader header;
        memset(&header, 0, sizeof(header));

        if (buffer.hasEnded() || !buffer.readLittle(header.signature)) {
            Logger::report(LogLevel_Error, boost::format("Failed to read directory header at 0x%x from file %s")
                                % buffer.getPosition() % file->getPath());
            return false;
        }
        if (header.signature != ZIPDirectoryHeader_Signature) {
            Logger::report(LogLevel_Error, boost::format("Invalid directory header signature 0x%x in file %s")
                                % header.signature % file->getPath());
            continue;
        }

        if (!buffer.readLittle(header.versionMade) ||
            !buffer.readLittle(header.versionRequired) ||
            !buffer.readLittle(header.flags) ||
            !buffer.readLittle(header.compressionMethod) ||
            !buffer.readLittle(header.modificationTime) ||
            !buffer.readLittle(header.modificationDate) ||
            !buffer.readLittle(header.crc32) ||
            !buffer.readLittle(header.compressedSize) ||
            !buffer.readLittle(header.uncompressedSize) ||
            !buffer.readLittle(header.nameLength) ||
            !buffer.readLittle(header.extrasLength) ||
            !buffer.readLittle(header.commentLength) ||
            !buffer.readLittle(header.diskStart) ||
            !buffer.readLittle(header.internalAttributes) ||
            !buffer.readLittle(header.externalAttributes) ||
            !buffer.readLittle(header.headerOffset)) {
            Logger::report(LogLevel_Error, boost::format("Failed to read directory header from file %s") % file->getPath());
            return false;
        }

        std::string name(buffer.at<char>(buffer.getPosition()), header.nameLength);
        if (name.empty()) {
            Logger::report(LogLevel_Error, boost::format("Found empty directory entry name in file %s") % file->getPath());
            continue;
        }

        ArchiveEntryPtr entry = boost::make_shared<ZIPArchiveEntry>(name.c_str(), file, header);
        if (!entry) {
            return false;
        }
        entries[name] = entry;

        if (!buffer.seek(header.nameLength + header.extrasLength + header.commentLength, BufferOrigin_Current)) {
            Logger::report(LogLevel_Error, boost::format("Failed to progress to the next directory header in file %s") % file->getPath());
            return false;
        }
    }

    Logger::report(LogLevel_Trace, boost::format("Found %d entries in file %s") % entries.size() % file->getPath());
    return true;
}
