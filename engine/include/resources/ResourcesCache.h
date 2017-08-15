/*
 * Copyright (c) 2017 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_RESOURCES_CACHE_H
#define ROS_RESOURCES_CACHE_H

#include <core/Common.h>
#include <core/Environment.h>
#include <core/Factory.h>
#include <application/Logger.h>
#include <resources/ResourceLoader.h>

namespace ros {
    class ResourcesCache;
    typedef boost::shared_ptr<ResourcesCache> ResourcesCachePtr;

    class ROS_API ResourcesCache : public boost::noncopyable {
            friend class Resource;

        public:
            static ResourcesCachePtr initInstance(const pt::ptree& config);
            static ResourcesCachePtr getInstance() { return instance; }

            virtual ~ResourcesCache() {}

            void addLoader(ResourceLoaderPtr loader);
            ResourceLoaderPtr findLoaderForResource(const std::string& name) const;

            virtual bool hasResource(const std::string& name) const =0;
            virtual void dropResource(const std::string& name) =0;
            virtual U32 getUsedSize() const =0;

            virtual void setCapacity(U32Opt capacity) =0;
            virtual U32Opt getCapacity() const =0;

            template<class T>
            boost::shared_ptr<T> acquireResource(const std::string& name) {
                ResourcePtr resource = this->loadResource(name);
                if (!resource) {
                    return boost::shared_ptr<T>();
                }
                boost::shared_ptr<T> casted = boost::dynamic_pointer_cast<T>(resource);
                if (!casted) {
                    ROS_ERROR(boost::format("Invalid type requested for resource %s") % name);
                    return boost::shared_ptr<T>();
                }
                return casted;
            }

        protected:
            virtual ResourcePtr loadResource(const std::string& name) =0;
            virtual void onResourceSizeChanged(S32 delta) =0;

        private:
            static Factory<ResourcesCache> factory;
            static ResourcesCachePtr instance;
            ResourceLoadersList loaders;
    };
}

#endif // ROS_RESOURCES_CACHE_H

