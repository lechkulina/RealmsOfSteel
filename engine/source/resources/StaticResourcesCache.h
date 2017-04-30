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
            StaticResourcesCache();

            virtual bool hasResource(const std::string& name) const;
            virtual void dropResource(const std::string& name);
            virtual U32 getUsedSize() const { return usedSize; }

            virtual void setCapacity(U32Opt capacity);
            virtual U32Opt getCapacity() const { return capacity; }

        protected:
            virtual ResourcePtr loadResource(const std::string& name);
            virtual void onResourceSizeChanged(S32 delta);

        private:
            U32Opt capacity;
            U32 usedSize;
            ResourcesMap resources;
    };
}
#endif // ROS_STATIC_RESOURCES_CACHE_H

