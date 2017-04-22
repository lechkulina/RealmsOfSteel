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
        public:
            static ResourcesCachePtr initInstance(const std::string& classId);
            static ResourcesCachePtr getInstance() { return instance; }

            virtual ~ResourcesCache() {}

            bool addLoader(ResourceLoaderPtr loader);
            ResourceLoaderPtr findLoader(const std::string& resourceName) const;

            virtual ResourcePtr readResource(const std::string& resourceName) =0;
            virtual bool hasResource(const std::string& resourceName) const =0;
            virtual U32 computeUsedSize() const =0;

            template<class T>
            boost::shared_ptr<T> fetchResource(const std::string& resourceName) {
                ResourcePtr resource = this->readResource(resourceName);
                if (!resource) {
                    return boost::shared_ptr<T>();
                }
                boost::shared_ptr<T> casted = boost::dynamic_pointer_cast<T>(resource);
                if (!casted) {
                    Logger::report(LogLevel_Error, boost::format("Invalid resource type used for resource %s") % resourceName);
                    return boost::shared_ptr<T>();
                }
                return casted;
            }

        private:
            static Factory<ResourcesCache> factory;
            static ResourcesCachePtr instance;
            ResourceLoadersList loaders;
    };
}

#endif // ROS_RESOURCES_CACHE_H

