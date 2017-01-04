/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_ARCHIVE_ENTRY_H
#define ROS_ARCHIVE_ENTRY_H

#include <core/Common.h>
#include <core/Environment.h>
#include <core/RawBuffer.h>

namespace ros {
    class ROS_API ArchiveEntry {
        public:
            virtual ~ArchiveEntry() {}

            virtual const std::string& getName() const =0;
            virtual U32 getCompressedSize() const =0;
            virtual U32 getUncompressedSize() const =0;
            virtual RawBufferPtr decompress() =0;
    };

    typedef boost::shared_ptr<ArchiveEntry> ArchiveEntryPtr;
    typedef std::map<std::string, ArchiveEntryPtr> ArchiveEntryMap;
}

#endif // ROS_ARCHIVE_ENTRY_H

