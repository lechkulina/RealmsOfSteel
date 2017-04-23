/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <zlib.h>
#include <application/Logger.h>
#include "ZIPArchiveEntry.h"

ros::ZIPArchiveEntry::ZIPArchiveEntry(const std::string& name, RawFilePtr file, const ZIPDirectoryHeader& directoryHeader)
    : name(name)
    , file(file)
    , directoryHeader(directoryHeader) {
}

bool ros::ZIPArchiveEntry::isCompressed() const {
    return fileHeader.compressionMethod == ZIPFileHeader::COMPRESSION_METHOD_NONE;
}

ros::RawBufferPtr ros::ZIPArchiveEntry::decompress() {
    if (!readFileHeader()) {
        return ros::RawBufferPtr();
    }
    switch(fileHeader.compressionMethod) {
        case ZIPFileHeader::COMPRESSION_METHOD_NONE:
            return readUncompressedData();

        case ZIPFileHeader::COMPRESSION_METHOD_DEFLATED:
            return inflateCompressedData();
    }
    Logger::report(LogLevel_Error, boost::format("Unsupported compression method used for entry %s in archive %s")
                        % getName() % file->getPath());
    return ros::RawBufferPtr();
}

bool ros::ZIPArchiveEntry::readFileHeader() {
    memset(&fileHeader, 0, sizeof(fileHeader));

    if (!file->seek(directoryHeader.headerOffset, FileOrigin_Begin) ||
        !file->readLittle(fileHeader.signature)) {
        Logger::report(LogLevel_Error, boost::format("Failed to read file header signature for entry %s from archive %s")
                            % getName() % file->getPath());
        return false;
    }
    if (fileHeader.signature != ZIPFileHeader::SIGNATURE) {
        Logger::report(LogLevel_Error, boost::format("Invalid file header signature set for entry %s in archive %s")
                            % getName() % file->getPath());
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
        Logger::report(LogLevel_Error, boost::format("Failed to read file header for entry %s from archive %s")
                            % getName() % file->getPath());
        return false;
    }

    return true;
}

ros::RawBufferPtr ros::ZIPArchiveEntry::readUncompressedData() {
    RawBufferPtr buffer = boost::make_shared<RawBuffer>(directoryHeader.uncompressedSize);
    if (!buffer || buffer->isNull()) {
        Logger::report(LogLevel_Error, boost::format("Failed to create destination buffer of size %d bytes for entry %s from archive %s")
                            % directoryHeader.uncompressedSize % getName() % file->getPath());
        return RawBufferPtr();
    }

    if (!file->seek(fileHeader.nameLength + fileHeader.extrasLength, FileOrigin_Current) ||
        !file->read(buffer->at<void>(), buffer->getSize())) {
        Logger::report(LogLevel_Error, boost::format("Failed to read uncompressed data for entry %s from archive %s")
                            % getName() % file->getPath());
        return RawBufferPtr();
    }

    return buffer;
}

ros::RawBufferPtr ros::ZIPArchiveEntry::inflateCompressedData() {
    RawBufferPtr dstBuffer = boost::make_shared<RawBuffer>(directoryHeader.uncompressedSize);
    if (!dstBuffer || dstBuffer->isNull()) {
        Logger::report(LogLevel_Error, boost::format("Failed to create destination buffer of size %d bytes for entry %s from archive %s")
                            % directoryHeader.uncompressedSize % getName() % file->getPath());
        return RawBufferPtr();
    }

    RawBuffer srcBuffer(directoryHeader.compressedSize);
    if (srcBuffer.isNull()) {
        Logger::report(LogLevel_Error, boost::format("Failed to create source buffer of size %d bytes for entry %s from archive %s")
                            % directoryHeader.compressedSize % getName() % file->getPath());
        return RawBufferPtr();
    }
    if (!file->seek(fileHeader.nameLength + fileHeader.extrasLength, FileOrigin_Current) ||
        !file->read(srcBuffer.at<void>(), srcBuffer.getSize())) {
        Logger::report(LogLevel_Error, boost::format("Failed to read compressed data for entry %s from archive %s")
                            % getName() % file->getPath());
        return RawBufferPtr();
    }

    z_stream decompressor;
    memset(&decompressor, 0, sizeof(z_stream));
    decompressor.next_in = srcBuffer.at<Bytef>();
    decompressor.avail_in = srcBuffer.getSize();
    decompressor.next_out = dstBuffer->at<Bytef>();
    decompressor.avail_out = dstBuffer->getSize();

    if (inflateInit2(&decompressor, -MAX_WBITS) != Z_OK) {
        Logger::report(LogLevel_Error, boost::format("Failed to create deflate decompressor for entry %s from archive %s - ZLIB error occured %s")
                            % getName() % file->getPath() % decompressor.msg);
        return RawBufferPtr();
    }

    int result = inflate(&decompressor, Z_FINISH);
    if (result != Z_OK && result != Z_STREAM_END) {
        Logger::report(LogLevel_Error, boost::format("Failed to decompress defalte data for entry %s from archive %s - ZLIB error occured %s")
                            % getName() % file->getPath() % decompressor.msg);
        return RawBufferPtr();
    }

    inflateEnd(&decompressor);
    return dstBuffer;
}
