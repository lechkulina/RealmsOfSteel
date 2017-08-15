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

ros::ZIPArchiveFile::ZIPArchiveFile(const fs::path& path) {
    open(path);
}

ros::ZIPArchiveFile::~ZIPArchiveFile() {
    close();
}

bool ros::ZIPArchiveFile::open(const fs::path& path) {
    close();
    if (!initFile(path) || !readEntries()) {
        close();
        return false;
    }
    return true;
}

void ros::ZIPArchiveFile::close() {
    entries.clear();
    file.reset();
}

bool ros::ZIPArchiveFile::isOpen() const {
    return file && file->isOpen();
}

const ros::fs::path& ros::ZIPArchiveFile::getPath() const {
    if (file) {
        return file->getPath();
    }
    return File::EMPTY_PATH;
}

bool ros::ZIPArchiveFile::initFile(const fs::path& path) {
    file = boost::make_shared<RawFile>(path, FileType_Binary, FileOpenMode_Read);
    if (!file || !file->isOpen()) {
        ROS_ERROR(boost::format("Failed to open archive %s") % path);
        return false;
    }
    return true;
}

bool ros::ZIPArchiveFile::readEntries() {
    ZIPDirectoryRecord record;
    memset(&record, 0, sizeof(record));

    U32 position = -sizeof(U32);
    while(true) {
        if (!file->seek(position, FileOrigin_End) ||
            !file->readLittle(record.signature)) {
            ROS_ERROR(boost::format("Failed to find directory record signature in archive %s") % getPath());
            return false;
        }
        if (record.signature == ZIPDirectoryRecord::SIGNATURE) {
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
        ROS_ERROR(boost::format("Failed to read directory record from archive %s") % getPath());
        return false;
    }

    RawBuffer buffer(record.directorySize);
    if (buffer.isNull() ||
        !file->seek(record.directoryOffset, FileOrigin_Begin) ||
        !file->read(buffer.at<void>(), buffer.getSize())) {
        ROS_ERROR(boost::format("Failed to read directory headers from archive %s") % getPath());
        return false;
    }

    for (U16 idx=0; idx < record.entriesCount; ++idx) {
        ZIPDirectoryHeader header;
        memset(&header, 0, sizeof(header));

        if (buffer.hasEnded() || !buffer.readLittle(header.signature)) {
            ROS_ERROR(boost::format("Failed to read directory header from archive %s") % getPath());
            return false;
        }
        if (header.signature != ZIPDirectoryHeader::SIGNATURE) {
            ROS_ERROR(boost::format("Invalid directory header signature in archive %s") % getPath());
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
            ROS_ERROR(boost::format("Failed to read directory header from archive %s") % getPath());
            return false;
        }

        std::string name(buffer.at<char>(buffer.getPosition()), header.nameLength);
        if (name.empty()) {
            ROS_WARNING(boost::format("Found empty entry name in archive %s") % getPath());
            continue;
        }

        ArchiveEntryPtr entry = boost::make_shared<ZIPArchiveEntry>(name, file, header);
        if (!entry) {
            return false;
        }
        entries[name] = entry;

        const S64 offset = header.nameLength + header.extrasLength + header.commentLength;
        if (!buffer.seek(offset, BufferOrigin_Current)) {
            ROS_ERROR(boost::format("Failed to seek to the next directory header in archive %s") % getPath());
            return false;
        }
    }

    ROS_DEBUG(boost::format("Found %d entries in archive %s") % entries.size() % getPath());
    return true;
}
