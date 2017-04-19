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

ros::ZIPArchiveFile::ZIPArchiveFile(const std::string& path) {
    open(path);
}

ros::ZIPArchiveFile::~ZIPArchiveFile() {
    close();
}

bool ros::ZIPArchiveFile::open(const std::string& path) {
    close();
    if (!initRawFile(path) || !readEntries()) {
        close();
        return false;
    }
    return true;
}

void ros::ZIPArchiveFile::close() {
    entries.clear();
    rawFile.reset();
}

bool ros::ZIPArchiveFile::isOpen() const {
    return rawFile && rawFile->isOpen();
}

const std::string& ros::ZIPArchiveFile::getPath() const {
    if (rawFile) {
        return rawFile->getPath();
    }
    return File::EMPTY_PATH;
}

bool ros::ZIPArchiveFile::initRawFile(const std::string& path) {
    rawFile = boost::make_shared<RawFile>(path.c_str(), FileType_Binary, FileOpenMode_Read);
    if (!rawFile || !rawFile->isOpen()) {
        Logger::report(LogLevel_Error, boost::format("Failed to open archive file %s") % path);
        return false;
    }
    return true;
}

bool ros::ZIPArchiveFile::readEntries() {
    ZIPDirectoryRecord directoryRecord;
    memset(&directoryRecord, 0, sizeof(directoryRecord));

    U32 directoryRecordPosition = -sizeof(U32);
    while(true) {
        if (!rawFile->seek(directoryRecordPosition, FileOrigin_End) ||
            !rawFile->readLittle(directoryRecord.signature)) {
            Logger::report(LogLevel_Error, boost::format("Failed to find directory record signature in archive file %s") % getPath());
            return false;
        }
        if (directoryRecord.signature == ZIPDirectoryRecord::SIGNATURE) {
            break;
        }
        --directoryRecordPosition;
    }

    if (!rawFile->readLittle(directoryRecord.diskNumber) ||
        !rawFile->readLittle(directoryRecord.diskStart) ||
        !rawFile->readLittle(directoryRecord.entriesCount) ||
        !rawFile->readLittle(directoryRecord.entriesTotalCount) ||
        !rawFile->readLittle(directoryRecord.directorySize) ||
        !rawFile->readLittle(directoryRecord.directoryOffset) ||
        !rawFile->readLittle(directoryRecord.commentLength)) {
        Logger::report(LogLevel_Error, boost::format("Failed to read directory record from archive file %s") % getPath());
        return false;
    }

    RawBuffer rawBuffer(directoryRecord.directorySize);
    if (rawBuffer.isNull() ||
        !rawFile->seek(directoryRecord.directoryOffset, FileOrigin_Begin) ||
        !rawFile->read(rawBuffer.at<void>(), rawBuffer.getSize())) {
        Logger::report(LogLevel_Error, boost::format("Failed to read directory headers from archive file %s") % getPath());
        return false;
    }

    for (U16 i=0; i < directoryRecord.entriesCount; ++i) {
        ZIPDirectoryHeader directoryHeader;
        memset(&directoryHeader, 0, sizeof(directoryHeader));

        if (rawBuffer.hasEnded() || !rawBuffer.readLittle(directoryHeader.signature)) {
            Logger::report(LogLevel_Error, boost::format("Failed to read directory header from archive file %s") % getPath());
            return false;
        }
        if (directoryHeader.signature != ZIPDirectoryHeader::SIGNATURE) {
            Logger::report(LogLevel_Error, boost::format("Invalid directory header signature in archive file %s") % getPath());
            continue;
        }

        if (!rawBuffer.readLittle(directoryHeader.versionMade) ||
            !rawBuffer.readLittle(directoryHeader.versionRequired) ||
            !rawBuffer.readLittle(directoryHeader.flags) ||
            !rawBuffer.readLittle(directoryHeader.compressionMethod) ||
            !rawBuffer.readLittle(directoryHeader.modificationTime) ||
            !rawBuffer.readLittle(directoryHeader.modificationDate) ||
            !rawBuffer.readLittle(directoryHeader.crc32) ||
            !rawBuffer.readLittle(directoryHeader.compressedSize) ||
            !rawBuffer.readLittle(directoryHeader.uncompressedSize) ||
            !rawBuffer.readLittle(directoryHeader.nameLength) ||
            !rawBuffer.readLittle(directoryHeader.extrasLength) ||
            !rawBuffer.readLittle(directoryHeader.commentLength) ||
            !rawBuffer.readLittle(directoryHeader.diskStart) ||
            !rawBuffer.readLittle(directoryHeader.internalAttributes) ||
            !rawBuffer.readLittle(directoryHeader.externalAttributes) ||
            !rawBuffer.readLittle(directoryHeader.headerOffset)) {
            Logger::report(LogLevel_Error, boost::format("Failed to read directory header from archive file %s") % getPath());
            return false;
        }

        std::string entryName(rawBuffer.at<char>(rawBuffer.getPosition()), directoryHeader.nameLength);
        if (entryName.empty()) {
            Logger::report(LogLevel_Error, boost::format("Found empty entry name in archive file %s") % getPath());
            continue;
        }

        ArchiveEntryPtr entry = boost::make_shared<ZIPArchiveEntry>(entryName, rawFile, directoryHeader);
        if (!entry) {
            return false;
        }
        entries[entryName] = entry;

        S64 nextEntryOffset = directoryHeader.nameLength + directoryHeader.extrasLength + directoryHeader.commentLength;
        if (!rawBuffer.seek(nextEntryOffset, BufferOrigin_Current)) {
            Logger::report(LogLevel_Error, boost::format("Failed to seek to the next directory header in archive file %s used for archive %s")
                                % getPath());
            return false;
        }
    }

    Logger::report(LogLevel_Trace, boost::format("Found %d entries in archive file %s") % entries.size() % getPath());
    return true;
}
