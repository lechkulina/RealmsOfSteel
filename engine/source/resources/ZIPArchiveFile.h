/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_ZIP_ARCHIVE_FILE_H
#define ROS_ZIP_ARCHIVE_FILE_H

#include <core/RawFile.h>
#include <resources/ArchiveFile.h>

namespace ros {
    struct ZIPDirectoryRecord {
        U32 signature;
        U16 diskNumber;
        U16 diskStart;
        U16 entriesCount;
        U16 entriesTotalCount;
        U32 directorySize;
        U32 directoryOffset;
        U16 commentLength;
    };

    struct ZIPDirectoryHeader {
        U32 signature;
        U16 versionMade;
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
        U16 commentLength;
        U16 diskStart;
        U16 internalAttributes;
        U32 externalAttributes;
        U32 headerOffset;
    };

    static const U32 ZIPDirectoryRecord_Signature = 0x06054b50;
    static const U32 ZIPDirectoryHeader_Signature = 0x02014b50;

    class ZIPArchiveFile : public ArchiveFile {
        public:
            ZIPArchiveFile() {}
            explicit ZIPArchiveFile(const char* path);
            virtual ~ZIPArchiveFile();

            virtual bool open(const char* path);
            virtual void close();
            virtual bool isOpen() const;
            virtual const std::string& getPath() const { return path; }

            virtual const ArchiveEntryMap& getEntries() const { return entries; }

        private:
            RawFilePtr file;
            std::string path;
            ArchiveEntryMap entries;

            bool initFile(const char* path);
            bool readEntries();
    };
}

#endif // ROS_ZIP_ARCHIVE_FILE_H

