/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_BUFFER_H
#define ROS_BUFFER_H

#include <core/Common.h>
#include <core/Environment.h>

namespace ros {
    enum BufferOrigin {
        BufferOrigin_Begin,
        BufferOrigin_Current,
        BufferOrigin_End
    };

    class ROS_API Buffer {
        public:
            virtual ~Buffer() {}

            virtual bool isNull() const =0;
            virtual U32 getSize() const =0;
    };

    typedef boost::shared_ptr<Buffer> BufferPtr;
}

#endif // ROS_BUFFER_H

