/*
 * Copyright (c) 2017 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_ARCHIVE_FILE_SYSTEM_H
#define ROS_ARCHIVE_FILE_SYSTEM_H

#include <core/Common.h>
#include <core/Environment.h>
#include <resources/FileSystem.h>
#include <resources/ArchiveFile.h>

namespace ros {
    class ROS_API ArchiveFileSystem : public FileSystem {
        public:
            ArchiveFileSystem();

            bool addArchiveFile(ArchiveFilePtr archiveFile);
            bool openArchiveFile(const std::string& path);

            virtual RawBufferPtr readFile(const std::string& fileName) const;
            virtual bool hasFile(const std::string& fileName) const;

        private:
            ArchiveFileList archiveFiles;
    };
}
#endif // ROS_ARCHIVE_FILE_SYSTEM_H

