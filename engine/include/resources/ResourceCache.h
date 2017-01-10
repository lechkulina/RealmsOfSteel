/*
 * Copyright (c) 2017 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_RESOURCE_CACHE_H
#define ROS_RESOURCE_CACHE_H

#include <core/Common.h>
#include <core/Environment.h>
#include <resources/ArchiveFileManager.h>
#include <resources/BufferLoaderManager.h>
#include <resources/BufferCacheManager.h>
#include <resources/BufferCache.h>

namespace ros {
    class ResourceCache;
    typedef boost::shared_ptr<ResourceCache> ResourceCachePtr;

    class ROS_API ResourceCache : public boost::noncopyable {
        public:
            static ResourceCachePtr initInstance(const PropertyTree& config);
            static ResourceCachePtr getInstance() { return instance; }

            ~ResourceCache();

            bool init(const PropertyTree& config);
            void uninit();

            BufferPtr acquireBuffer(const std::string& name);
            void releaseBuffer(BufferPtr buffer);

            ArchiveFileManager& getArchiveManager() { return archiveManager; }
            BufferLoaderManager& getLoaderManager() { return loaderManager; }
            BufferCacheManager& getCacheManager() { return cacheManager; }
            BufferCacheList& getCaches() { return caches; }

        private:
            static ResourceCachePtr instance;
            ArchiveFileManager archiveManager;
            BufferLoaderManager loaderManager;
            BufferCacheManager cacheManager;
            BufferCacheList caches;
    };
}

#endif // ROS_RESOURCE_CACHE_H

