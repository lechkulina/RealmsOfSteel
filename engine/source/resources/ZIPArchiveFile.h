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
        static const U32 SIGNATURE = 0x06054b50;

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
        static const U32 SIGNATURE = 0x02014b50;

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

    class ZIPArchiveFile : public ArchiveFile {
        public:
            ZIPArchiveFile() {}
            explicit ZIPArchiveFile(const fs::path& path);
            virtual ~ZIPArchiveFile();

            virtual bool open(const fs::path& path);
            virtual void close();
            virtual bool isOpen() const;
            virtual const fs::path& getPath() const;

            virtual const ArchiveEntriesMap& getEntries() const { return entries; }

        private:
            RawFilePtr file;
            ArchiveEntriesMap entries;

            bool initFile(const fs::path& path);
            bool readEntries();
    };
}

#endif // ROS_ZIP_ARCHIVE_FILE_H

