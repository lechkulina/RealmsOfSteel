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
            bool addArchive(ArchiveFilePtr archive);
            bool openArchive(const fs::path& path);

            virtual bool setRoot(const fs::path& path);
            virtual const fs::path& getRoot() const { return root; }
            virtual char getSeparator() const { return '/'; }
            virtual RawBufferPtr readFile(const std::string& name) const;
            virtual bool hasFile(const std::string& name) const;

        private:
            fs::path root;
            ArchiveFilesList archives;
    };
}
#endif // ROS_ARCHIVE_FILE_SYSTEM_H

