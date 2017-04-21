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
    typedef std::list<ArchiveFilePtr> ArchiveFileList;
    typedef std::map<std::string, ArchiveFilePtr> ArchiveFileMap;

    class ROS_API ArchiveFile: public File {
        public:
            static ArchiveFilePtr create(const std::string& classId);

            virtual bool open(const std::string& path) =0;
            virtual const ArchiveEntriesMap& getEntries() const =0;

            ArchiveEntryPtr findEntry(const std::string& entryName) const;
            bool hasEntry(const std::string& entryName) const;

        private:
            static Factory<ArchiveFile> factory;
    };
}

#endif // ROS_ARCHIVE_FILE_H

