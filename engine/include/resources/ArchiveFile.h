/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_ARCHIVE_FILE_H
#define ROS_ARCHIVE_FILE_H

#include <core/Common.h>
#include <core/Environment.h>
#include <core/Factory.h>
#include <core/File.h>
#include <resources/ArchiveEntry.h>

namespace ros {
    class ArchiveFile;
    typedef boost::shared_ptr<ArchiveFile> ArchiveFilePtr;
    typedef Factory<ArchiveFile> ArchiveFileFactory;

    class ROS_API ArchiveFile: public File {
        public:
            static ArchiveFilePtr create(const char* path);

            virtual ~ArchiveFile() {}

            virtual bool open(const char* path) =0;
            virtual void close() =0;

            virtual const ArchiveEntryMap& getEntries() const =0;

        private:
            static ArchiveFileFactory factory;
    };
}

#endif // ROS_ARCHIVE_FILE_H

