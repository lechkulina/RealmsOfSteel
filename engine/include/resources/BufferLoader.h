/*
 * Copyright (c) 2017 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_BUFFER_LOADER_H
#define ROS_BUFFER_LOADER_H

#include <core/Common.h>
#include <core/Environment.h>
#include <core/Factory.h>
#include <core/RawBuffer.h>

namespace ros {
    class ROS_API BufferLoader : public boost::noncopyable {
        public:
            virtual ~BufferLoader() {}

            virtual bool init(const PropertyTree& config);
            virtual void uninit();

            virtual bool isLoadable(const std::string& name) const =0;
            virtual BufferPtr loadBuffer(RawBufferPtr src) =0;

            const std::string& getName() const { return name; }

        private:
            std::string name;
    };

    typedef boost::shared_ptr<BufferLoader> BufferLoaderPtr;
    typedef Factory<BufferLoader> BufferLoaderFactory;
    typedef std::list<BufferLoaderPtr> BufferLoaderList;
    typedef std::map<std::string, BufferLoaderPtr> BufferLoaderMap;
}

#endif // ROS_BUFFER_LOADER_H

