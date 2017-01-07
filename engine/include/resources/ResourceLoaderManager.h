/*
 * Copyright (c) 2017 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_RESOURCE_LOADER_MANAGER_H
#define ROS_RESOURCE_LOADER_MANAGER_H

#include <core/Common.h>
#include <core/Environment.h>
#include <resources/ResourceLoader.h>

namespace ros {
    class ResourceLoaderManager;
    typedef boost::shared_ptr<ResourceLoaderManager> ResourceLoaderManagerPtr;

    class ROS_API ResourceLoaderManager: public boost::noncopyable {
        public:
            static ResourceLoaderManagerPtr initInstance(const PropertyTree& config);
            static ResourceLoaderManagerPtr getInstance() { return manager; }

            ~ResourceLoaderManager();

            bool init(const PropertyTree& config);
            void uninit();

            ResourceLoaderPtr initLoader(const PropertyTree& config);

        private:
            static ResourceLoaderManagerPtr manager;
            ResourceLoaderFactory factory;
            ResourceLoaderMap loaders;
    };
}

#endif // ROS_RESOURCE_LOADER_MANAGER_H

