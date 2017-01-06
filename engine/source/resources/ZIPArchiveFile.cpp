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

ros::ZIPArchiveFile::ZIPArchiveFile(const PropertyTree& config) {
    open(config);
}

ros::ZIPArchiveFile::~ZIPArchiveFile() {
    close();
}

bool ros::ZIPArchiveFile::open(const PropertyTree& config) {
    close();
    if (!ArchiveFile::open(config) || !initFile(config) || !readEntries()) {
        close();
        return false;
    }
    return true;
}

void ros::ZIPArchiveFile::close() {
    entries.clear();
    path.clear();
    file.reset();
    ArchiveFile::close();
}

bool ros::ZIPArchiveFile::isOpen() const {
    return file && file->isOpen();
}

bool ros::ZIPArchiveFile::initFile(const PropertyTree& config) {
    StringOpt path = config.get_optional<std::string>("path");
    if (!path) {
        Logger::report(LogLevel_Error, boost::format("Missing path property in archive %s") % getName());
        return false;
    }
    this->path = *path;

    file = boost::make_shared<RawFile>(path->c_str(), FileType_Binary, FileOpenMode_Read);
    if (!file || !file->isOpen()) {
        Logger::report(LogLevel_Error, boost::format("Failed to open file %s for archive %s") % path % getName());
        return false;
    }

    return true;
}

bool ros::ZIPArchiveFile::readEntries() {
    ZIPDirectoryRecord record;
    memset(&record, 0, sizeof(record));

    U32 position = -sizeof(U32);
    while(true) {
        if (!file->seek(position, FileOrigin_End) || !file->readLittle(record.signature)) {
            Logger::report(LogLevel_Error, boost::format("Failed to find directory record signature 0x%x in file %s used for archive %s")
                                % ZIPDirectoryRecord_Signature % file->getPath() % getName());
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
        Logger::report(LogLevel_Error, boost::format("Failed to read directory record from file %s used for archive %s")
                            % file->getPath() % getName());
        return false;
    }

    RawBuffer buffer(record.directorySize);
    if (buffer.isNull() || !file->seek(record.directoryOffset, FileOrigin_Begin) || !file->read(buffer.at<void>(), buffer.getSize())) {
        Logger::report(LogLevel_Error, boost::format("Failed to read directory headers from file %s used for archive %s")
                            % file->getPath() % getName());
        return false;
    }

    for (U16 i=0; i < record.entriesCount; ++i) {
        ZIPDirectoryHeader header;
        memset(&header, 0, sizeof(header));

        if (buffer.hasEnded() || !buffer.readLittle(header.signature)) {
            Logger::report(LogLevel_Error, boost::format("Failed to read directory header at 0x%x from file %s used for archive %s")
                                % buffer.getPosition() % file->getPath() % getName());
            return false;
        }
        if (header.signature != ZIPDirectoryHeader_Signature) {
            Logger::report(LogLevel_Error, boost::format("Invalid directory header signature 0x%x in file %s used for archive %s")
                                % header.signature % file->getPath() % getName());
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
            Logger::report(LogLevel_Error, boost::format("Failed to read directory header from file %s used for archive %s")
                                % file->getPath() % getName());
            return false;
        }

        std::string entryName(buffer.at<char>(buffer.getPosition()), header.nameLength);
        if (entryName.empty()) {
            Logger::report(LogLevel_Error, boost::format("Found empty entry name in file %s used for archive %s")
                                % file->getPath() % getName());
            continue;
        }

        ArchiveEntryPtr entry = boost::make_shared<ZIPArchiveEntry>(entryName.c_str(), file, header);
        if (!entry) {
            return false;
        }
        entries[entryName] = entry;

        if (!buffer.seek(header.nameLength + header.extrasLength + header.commentLength, BufferOrigin_Current)) {
            Logger::report(LogLevel_Error, boost::format("Failed to progress to the next directory header in file %s used for archive %s")
                                % file->getPath() % getName());
            return false;
        }
    }

    Logger::report(LogLevel_Trace, boost::format("Found %d entries in file %s used for archive %s")
                        % entries.size() % file->getPath() % getName());
    return true;
}
