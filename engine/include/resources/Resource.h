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
    class ResourcesCache;
    typedef boost::weak_ptr<ResourcesCache> ResourcesCacheWeakPtr;

    class ROS_API Resource {
        public:
            Resource();
            virtual ~Resource() {}

            void setCache(ResourcesCacheWeakPtr cache);
            ResourcesCacheWeakPtr getCache() const { return cache; }
            U32 getSize() const { return size; }

            virtual bool isNull() const =0;

        protected:
            virtual void setSize(U32 size);

        private:
            ResourcesCacheWeakPtr cache;
            U32 size;
    };

    typedef boost::shared_ptr<Resource> ResourcePtr;
    typedef std::map<std::string, ResourcePtr> ResourcesMap;
}

#endif // ROS_RESOURCE_H

