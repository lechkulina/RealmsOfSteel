/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_ZIP_ARCHIVE_ENTRY_H
#define ROS_ZIP_ARCHIVE_ENTRY_H

#include <resources/ArchiveEntry.h>
#include "ZIPArchiveFile.h"

namespace ros {
    struct ZIPFileHeader {
        static const U32 SIGNATURE = 0x04034b50;

        static const U16 COMPRESSION_METHOD_NONE = 0;
        static const U16 COMPRESSION_METHOD_DEFLATED = 8;

        U32 signature;
        U16 versionRequired;
        U16 flags;
        U16 compressionMethod;
        U16 modificationTime;
        U16 modificationDate;
        U32 crc32;
        U32 compressedSize;
        U32 uncompressedSize;
        U16 nameLength;
        U16 extrasLength;
    };

    class ZIPArchiveEntry : public ArchiveEntry {
        public:
            ZIPArchiveEntry(const std::string& name, RawFilePtr rawFile, const ZIPDirectoryHeader& directoryHeader);

            virtual const std::string& getName() const { return name; }
            virtual U32 getCompressedSize() const { return directoryHeader.compressedSize; }
            virtual U32 getUncompressedSize() const { return directoryHeader.uncompressedSize; }
            virtual bool isCompressed() const;
            virtual RawBufferPtr decompress();

        private:
            std::string name;
            RawFilePtr rawFile;
            ZIPDirectoryHeader directoryHeader;
            ZIPFileHeader fileHeader;

            bool readFileHeader();
            RawBufferPtr readUncompressedData();
            RawBufferPtr inflateCompressedData();
    };
}

#endif // ROS_ZIP_ARCHIVE_ENTRY_H

