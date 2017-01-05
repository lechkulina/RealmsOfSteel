/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <zlib.h>
#include <application/Logger.h>
#include "ZIPArchiveEntry.h"

ros::ZIPArchiveEntry::ZIPArchiveEntry(const char* name, RawFilePtr file, const ZIPDirectoryHeader& directoryHeader)
    : name(name)
    , file(file)
    , directoryHeader(directoryHeader) {
}

ros::RawBufferPtr ros::ZIPArchiveEntry::decompress() {
    if (!readFileHeader()) {
        return ros::RawBufferPtr();
    }

    switch(fileHeader.compressionMethod) {
        case ZIPCompressionMethod_None:
            return readUncompressedData();

        case ZIPCompressionMethod_Deflated:
            return inflateCompressedData();
    }
    Logger::report(LogLevel_Error, boost::format("Unsupported compression method 0x%x used for entry %s in file %s")
                        % fileHeader.compressionMethod % name % file->getPath());
    return ros::RawBufferPtr();
}

bool ros::ZIPArchiveEntry::readFileHeader() {
    memset(&fileHeader, 0, sizeof(fileHeader));

    if (!file->seek(directoryHeader.headerOffset, FileOrigin_Begin) || !file->readLittle(fileHeader.signature)) {
        Logger::report(LogLevel_Error, boost::format("Failed to read file header signature at 0x%x for entry %s from file %s")
                            % file->getPosition() % name % file->getPath());
        return false;
    }
    if (fileHeader.signature != ZIPFileHeader_Signature) {
        Logger::report(LogLevel_Error, boost::format("Invalid file header signature 0x%x set for entry %s in file %s")
                            % fileHeader.signature % name % file->getPath());
        return false;
    }

    if (!file->readLittle(fileHeader.versionRequired) ||
        !file->readLittle(fileHeader.flags) ||
        !file->readLittle(fileHeader.compressionMethod) ||
        !file->readLittle(fileHeader.modificationTime) ||
        !file->readLittle(fileHeader.modificationDate) ||
        !file->readLittle(fileHeader.crc32) ||
        !file->readLittle(fileHeader.compressedSize) ||
        !file->readLittle(fileHeader.uncompressedSize) ||
        !file->readLittle(fileHeader.nameLength) ||
        !file->readLittle(fileHeader.extrasLength)) {
        Logger::report(LogLevel_Error, boost::format("Failed to read file header at 0x%x for entry %s from file %s")
                            % file->getPosition() % name % file->getPath());
        return false;
    }

    return true;
}

ros::RawBufferPtr ros::ZIPArchiveEntry::readUncompressedData() {
    RawBufferPtr dst = boost::make_shared<RawBuffer>(directoryHeader.uncompressedSize);
    if (!dst || dst->isNull()) {
        Logger::report(LogLevel_Error, boost::format("Failed to create destination buffer of size %d bytes for entry %s from file %s")
                            % directoryHeader.uncompressedSize % name % file->getPath());
        return RawBufferPtr();
    }

    if (!file->seek(fileHeader.nameLength + fileHeader.extrasLength, FileOrigin_Current) ||
        !file->read(dst->at<void>(), dst->getSize())) {
        Logger::report(LogLevel_Error, boost::format("Failed to read uncompressed data for entry %s from file %s")
                            % name % file->getPath());
        return RawBufferPtr();
    }

    return dst;
}

ros::RawBufferPtr ros::ZIPArchiveEntry::inflateCompressedData() {
    RawBufferPtr dst = boost::make_shared<RawBuffer>(directoryHeader.uncompressedSize);
    if (!dst || dst->isNull()) {
        Logger::report(LogLevel_Error, boost::format("Failed to create destination buffer of size %d bytes for entry %s from file %s")
                            % directoryHeader.uncompressedSize % name % file->getPath());
        return RawBufferPtr();
    }

    RawBuffer src(directoryHeader.compressedSize);
    if (src.isNull()) {
        Logger::report(LogLevel_Error, boost::format("Failed to create source buffer of size %d bytes for entry %s")
                            % directoryHeader.compressedSize % name);
        return RawBufferPtr();
    }
    if (!file->seek(fileHeader.nameLength + fileHeader.extrasLength, FileOrigin_Current) ||
        !file->read(src.at<void>(), src.getSize())) {
        Logger::report(LogLevel_Error, boost::format("Failed to read compressed data at 0x%x for entry %s from file %s")
                            % file->getPosition() % name % file->getPath());
        return RawBufferPtr();
    }

    z_stream decompressor;
    memset(&decompressor, 0, sizeof(z_stream));
    decompressor.next_in = src.at<Bytef>();
    decompressor.avail_in = src.getSize();
    decompressor.next_out = dst->at<Bytef>();
    decompressor.avail_out = dst->getSize();

    if (inflateInit2(&decompressor, -MAX_WBITS) != Z_OK) {
        Logger::report(LogLevel_Error, boost::format("Failed to create deflate decompressor for entry %s from file %s - ZLIB error occured %s")
                            % name % file->getPath() % decompressor.msg);
        return RawBufferPtr();
    }

    int result = inflate(&decompressor, Z_FINISH);
    if (result != Z_OK && result != Z_STREAM_END) {
        Logger::report(LogLevel_Error, boost::format("Failed to decompress defalte data for entry %s from file %s - ZLIB error occured %s")
                            % name % file->getPath() % decompressor.msg);
        return RawBufferPtr();
    }

    inflateEnd(&decompressor);
    return dst;
}
