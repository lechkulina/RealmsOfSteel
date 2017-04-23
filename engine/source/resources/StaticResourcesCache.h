/*
 * Copyright (c) 2017 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_STATIC_RESOURCES_CACHE_H
#define ROS_STATIC_RESOURCES_CACHE_H

#include <resources/ResourcesCache.h>

namespace ros {
    class StaticResourcesCache : public ResourcesCache {
        public:
            virtual ResourcePtr readResource(const std::string& name);
            virtual bool hasResource(const std::string& name) const;
            virtual U32 computeUsedSize() const;

        private:
            U32Opt capacity;
            ResourcesMap resources;
    };
}
#endif // ROS_STATIC_RESOURCES_CACHE_H

