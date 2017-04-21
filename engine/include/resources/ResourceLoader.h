/*
 * Copyright (c) 2017 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_RESOURCE_LOADER_H
#define ROS_RESOURCE_LOADER_H

#include <core/Common.h>
#include <core/Environment.h>
#include <resources/Resource.h>

namespace ros {
    class ResourceLoader;
    typedef boost::shared_ptr<ResourceLoader> ResourceLoaderPtr;
    typedef std::list<ResourceLoaderPtr> ResourceLoadersList;
    typedef std::map<std::string, ResourceLoaderPtr> ResourceLoadersMap;

    class ROS_API ResourceLoader: public boost::noncopyable {
        public:
            static ResourceLoaderPtr create(const std::string& classId);

            virtual ~ResourceLoader() {}

            virtual bool isLoadable(const std::string& resourceName) const =0;
            virtual ResourcePtr load(const std::string& resourceName) =0;
    };
}

#endif // ROS_RESOURCE_LOADER_H

