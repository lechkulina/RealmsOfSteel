/*
 * Copyright (c) 2017 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_BUFFER_LOADER_MANAGER_H
#define ROS_BUFFER_LOADER_MANAGER_H

#include <core/Common.h>
#include <core/Environment.h>
#include <resources/BufferLoader.h>

namespace ros {
    class ROS_API BufferLoaderManager: public boost::noncopyable {
        public:
            ~BufferLoaderManager();

            bool init(const PropertyTree& config);
            void uninit();

            BufferLoaderPtr initLoader(const PropertyTree& config);
            bool initLoaders(const PropertyTree& config, BufferLoaderList& dst);

        private:
            BufferLoaderFactory factory;
            BufferLoaderMap loaders;
    };
}

#endif // ROS_BUFFER_LOADER_MANAGER_H

