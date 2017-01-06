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
    class ROS_API ArchiveFile: public File {
        public:
            virtual ~ArchiveFile() {}

            virtual bool open(const PropertyTree& config);
            virtual void close();

            virtual const ArchiveEntryMap& getEntries() const =0;

            const std::string& getName() const { return name; }

        private:
            std::string name;
    };

    typedef boost::shared_ptr<ArchiveFile> ArchiveFilePtr;
    typedef Factory<ArchiveFile> ArchiveFileFactory;
    typedef std::map<std::string, ArchiveFilePtr> ArchiveFileMap;
}

#endif // ROS_ARCHIVE_FILE_H

