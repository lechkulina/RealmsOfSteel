/*
 * Copyright (c) 2017 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_RESOURCE_CACHE_MANAGER_H
#define ROS_RESOURCE_CACHE_MANAGER_H

#include <core/Common.h>
#include <core/Environment.h>
#include <resources/ResourceCache.h>

namespace ros {
    class ResourceCacheManager;
    typedef boost::shared_ptr<ResourceCacheManager> ResourceCacheManagerPtr;

    class ROS_API ResourceCacheManager: public boost::noncopyable {
        public:
            static ResourceCacheManagerPtr initInstance(const PropertyTree& config);
            static ResourceCacheManagerPtr getInstance() { return manager; }

            ~ResourceCacheManager();

            bool init(const PropertyTree& config);
            void uninit();

            ResourceCachePtr initCache(const PropertyTree& config);
            bool initCaches(const PropertyTree& config, ResourceCacheList& dst);

        private:
            static ResourceCacheManagerPtr manager;
            ResourceCacheFactory factory;
            ResourceCacheMap caches;
    };
}

#endif // ROS_RESOURCE_CACHE_MANAGER_H

