/*
 * Copyright (c) 2017 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_RESOURCE_H
#define ROS_RESOURCE_H

#include <core/Common.h>
#include <core/Environment.h>

namespace ros {
    class ROS_API Resource {
        public:
            virtual ~Resource() {}

            virtual bool isNull() const =0;
            virtual U32 getSize() const =0;
    };

    typedef boost::shared_ptr<Resource> ResourcePtr;
    typedef std::map<std::string, ResourcePtr> ResourcesMap;
}

#endif // ROS_RESOURCE_H

