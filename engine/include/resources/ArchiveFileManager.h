/*
 * Copyright (c) 2017 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_ARCHIVE_FILE_MANAGER_H
#define ROS_ARCHIVE_FILE_MANAGER_H

#include <core/Common.h>
#include <core/Environment.h>
#include <resources/ArchiveFile.h>

namespace ros {
    class ArchiveFileManager;
    typedef boost::shared_ptr<ArchiveFileManager> ArchiveFileManagerPtr;

    class ROS_API ArchiveFileManager: public boost::noncopyable {
        public:
            static ArchiveFileManagerPtr initInstance(const PropertyTree& config);
            static ArchiveFileManagerPtr getInstance() { return manager; }

            ~ArchiveFileManager();

            bool init(const PropertyTree& config);
            void uninit();

            ArchiveFilePtr openArchive(const PropertyTree& config);
            bool openArchives(const PropertyTree& config, ArchiveFileList& dst);

        private:
            static ArchiveFileManagerPtr manager;
            ArchiveFileFactory factory;
            ArchiveFileMap archives;
    };
}

#endif // ROS_ARCHIVE_FILE_MANAGER_H

