/*
 * Copyright (c) 2017 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_BUFFER_CACHE_MANAGER_H
#define ROS_BUFFER_CACHE_MANAGER_H

#include <core/Common.h>
#include <core/Environment.h>
#include <resources/BufferCache.h>

namespace ros {
    class BufferCacheManager;
    typedef boost::shared_ptr<BufferCacheManager> BufferCacheManagerPtr;

    class ROS_API BufferCacheManager: public boost::noncopyable {
        public:
            static BufferCacheManagerPtr initInstance(const PropertyTree& config);
            static BufferCacheManagerPtr getInstance() { return manager; }

            ~BufferCacheManager();

            bool init(const PropertyTree& config);
            void uninit();

            BufferCachePtr initCache(const PropertyTree& config);
            bool initCaches(const PropertyTree& config, BufferCacheList& dst);

        private:
            static BufferCacheManagerPtr manager;
            BufferCacheFactory factory;
            BufferCacheMap caches;
    };
}

#endif // ROS_BUFFER_CACHE_MANAGER_H

