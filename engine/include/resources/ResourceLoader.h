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
#include <core/Factory.h>
#include <core/RawBuffer.h>

namespace ros {
    class ROS_API ResourceLoader : public boost::noncopyable {
        public:
            virtual ~ResourceLoader() {}

            virtual bool init(const PropertyTree& config);
            virtual void uninit();

            virtual BufferPtr load(RawBufferPtr src) =0;

            const std::string& getName() const { return name; }

        private:
            std::string name;
    };

    typedef boost::shared_ptr<ResourceLoader> ResourceLoaderPtr;
    typedef Factory<ResourceLoader> ResourceLoaderFactory;
    typedef std::list<ResourceLoaderPtr> ResourceLoaderList;
    typedef std::map<std::string, ResourceLoaderPtr> ResourceLoaderMap;
}

#endif // ROS_RESOURCE_LOADER_H

