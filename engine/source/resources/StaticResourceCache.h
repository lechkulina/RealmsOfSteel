/*
 * Copyright (c) 2017 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_STATIC_RESOURCE_CACHE_H
#define ROS_STATIC_RESOURCE_CACHE_H

#include <resources/ResourceCache.h>

namespace ros {
    class StaticResourceCache : public ResourceCache {
        public:
            virtual ~StaticResourceCache();

            virtual bool init(const PropertyTree& config);
            virtual void uninit();

            virtual BufferPtr acquireBuffer(const std::string& name);
            virtual void releaseBuffer(BufferPtr) {}
            virtual U32 computeUsedSize() const;

        private:
            U32Opt capacity;
            BufferMap buffers;
    };
}
#endif // ROS_STATIC_RESOURCE_CACHE_H

