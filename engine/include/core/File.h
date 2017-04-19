/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_FILE_H
#define ROS_FILE_H

#include <core/Common.h>
#include <core/Environment.h>

namespace ros {
    enum FileType {
        FileType_Invalid,
        FileType_Text,
        FileType_Binary
    };

    enum FileOpenMode {
        FileOpenMode_Invalid,
        FileOpenMode_Read,
        FileOpenMode_Write,
        FileOpenMode_Append,
        FileOpenMode_ReadUpdate,
        FileOpenMode_WriteUpdate,
        FileOpenMode_AppendUpdate
    };

    enum FileBufferMode {
        FileBufferMode_Invalid,
        FileBufferMode_Default,
        FileBufferMode_Full,
        FileBufferMode_Line,
        FileBufferMode_None
    };

    enum FileOrigin {
        FileOrigin_Begin,
        FileOrigin_Current,
        FileOrigin_End
    };

    class ROS_API File : public boost::noncopyable {
        public:
            static const std::string EMPTY_PATH;

            virtual ~File() {}

            virtual void close() =0;
            virtual bool isOpen() const =0;
            virtual const std::string& getPath() const =0;
    };

    typedef boost::shared_ptr<File> FilePtr;
}

#endif // ROS_FILE_H

