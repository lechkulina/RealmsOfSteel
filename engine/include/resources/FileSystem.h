/*
 * Copyright (c) 2017 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_FILE_SYSTEM_H
#define ROS_FILE_SYSTEM_H

#include <core/Common.h>
#include <core/Environment.h>
#include <core/Factory.h>
#include <core/RawBuffer.h>

namespace ros {
    class FileSystem;
    typedef boost::shared_ptr<FileSystem> FileSystemPtr;

    class ROS_API FileSystem: public boost::noncopyable {
        public:
            static FileSystemPtr initInstance(const std::string& classId);
            static FileSystemPtr getInstance() { return fileSystem; }

            virtual ~FileSystem() {}

            virtual RawBufferPtr readFile(const std::string& fileName) const =0;
            virtual bool hasFile(const std::string& fileName) const =0;

        private:
            static Factory<FileSystem> factory;
            static FileSystemPtr fileSystem;
    };
}

#endif // ROS_FILE_SYSTEM_H

