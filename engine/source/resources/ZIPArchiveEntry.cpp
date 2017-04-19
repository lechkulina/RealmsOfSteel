/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <zlib.h>
#include <application/Logger.h>
#include "ZIPArchiveEntry.h"

ros::ZIPArchiveEntry::ZIPArchiveEntry(const std::string& name, RawFilePtr rawFile, const ZIPDirectoryHeader& directoryHeader)
    : name(name)
    , rawFile(rawFile)
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
    Logger::report(LogLevel_Error, boost::format("Unsupported compression method used for entry %s in archive file %s")
                        % getName() % rawFile->getPath());
    return ros::RawBufferPtr();
}

bool ros::ZIPArchiveEntry::readFileHeader() {
    memset(&fileHeader, 0, sizeof(fileHeader));

    if (!rawFile->seek(directoryHeader.headerOffset, FileOrigin_Begin) ||
        !rawFile->readLittle(fileHeader.signature)) {
        Logger::report(LogLevel_Error, boost::format("Failed to read file header signature for entry %s from archive file %s")
                            % getName() % rawFile->getPath());
        return false;
    }
    if (fileHeader.signature != ZIPFileHeader::SIGNATURE) {
        Logger::report(LogLevel_Error, boost::format("Invalid file header signature set for entry %s in archive file %s")
                            % getName() % rawFile->getPath());
        return false;
    }

    if (!rawFile->readLittle(fileHeader.versionRequired) ||
        !rawFile->readLittle(fileHeader.flags) ||
        !rawFile->readLittle(fileHeader.compressionMethod) ||
        !rawFile->readLittle(fileHeader.modificationTime) ||
        !rawFile->readLittle(fileHeader.modificationDate) ||
        !rawFile->readLittle(fileHeader.crc32) ||
        !rawFile->readLittle(fileHeader.compressedSize) ||
        !rawFile->readLittle(fileHeader.uncompressedSize) ||
        !rawFile->readLittle(fileHeader.nameLength) ||
        !rawFile->readLittle(fileHeader.extrasLength)) {
        Logger::report(LogLevel_Error, boost::format("Failed to read file header for entry %s from archive file %s")
                            % getName() % rawFile->getPath());
        return false;
    }

    return true;
}

ros::RawBufferPtr ros::ZIPArchiveEntry::readUncompressedData() {
    RawBufferPtr dstBuffer = boost::make_shared<RawBuffer>(directoryHeader.uncompressedSize);
    if (!dstBuffer || dstBuffer->isNull()) {
        Logger::report(LogLevel_Error, boost::format("Failed to create destination buffer of size %d bytes for entry %s from archive file %s")
                            % directoryHeader.uncompressedSize % getName() % rawFile->getPath());
        return RawBufferPtr();
    }

    if (!rawFile->seek(fileHeader.nameLength + fileHeader.extrasLength, FileOrigin_Current) ||
        !rawFile->read(dstBuffer->at<void>(), dstBuffer->getSize())) {
        Logger::report(LogLevel_Error, boost::format("Failed to read uncompressed data for entry %s from archive file %s")
                            % getName() % rawFile->getPath());
        return RawBufferPtr();
    }

    return dstBuffer;
}

ros::RawBufferPtr ros::ZIPArchiveEntry::inflateCompressedData() {
    RawBufferPtr dstBuffer = boost::make_shared<RawBuffer>(directoryHeader.uncompressedSize);
    if (!dstBuffer || dstBuffer->isNull()) {
        Logger::report(LogLevel_Error, boost::format("Failed to create destination buffer of size %d bytes for entry %s from archive file %s")
                            % directoryHeader.uncompressedSize % getName() % rawFile->getPath());
        return RawBufferPtr();
    }

    RawBuffer srcBuffer(directoryHeader.compressedSize);
    if (srcBuffer.isNull()) {
        Logger::report(LogLevel_Error, boost::format("Failed to create source buffer of size %d bytes for entry %s from archive file %s")
                            % directoryHeader.compressedSize % getName() % rawFile->getPath());
        return RawBufferPtr();
    }
    if (!rawFile->seek(fileHeader.nameLength + fileHeader.extrasLength, FileOrigin_Current) ||
        !rawFile->read(srcBuffer.at<void>(), srcBuffer.getSize())) {
        Logger::report(LogLevel_Error, boost::format("Failed to read compressed data for entry %s from archive file %s")
                            % getName() % rawFile->getPath());
        return RawBufferPtr();
    }

    z_stream decompressor;
    memset(&decompressor, 0, sizeof(z_stream));
    decompressor.next_in = srcBuffer.at<Bytef>();
    decompressor.avail_in = srcBuffer.getSize();
    decompressor.next_out = dstBuffer->at<Bytef>();
    decompressor.avail_out = dstBuffer->getSize();

    if (inflateInit2(&decompressor, -MAX_WBITS) != Z_OK) {
        Logger::report(LogLevel_Error, boost::format("Failed to create deflate decompressor for entry %s from archive file %s - ZLIB error occured %s")
                            % getName() % rawFile->getPath() % decompressor.msg);
        return RawBufferPtr();
    }

    int result = inflate(&decompressor, Z_FINISH);
    if (result != Z_OK && result != Z_STREAM_END) {
        Logger::report(LogLevel_Error, boost::format("Failed to decompress defalte data for entry %s from archive file %s - ZLIB error occured %s")
                            % getName() % rawFile->getPath() % decompressor.msg);
        return RawBufferPtr();
    }

    inflateEnd(&decompressor);
    return dstBuffer;
}
