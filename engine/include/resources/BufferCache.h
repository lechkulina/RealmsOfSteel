/*
 * Copyright (c) 2017 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_BUFFER_CACHE_H
#define ROS_BUFFER_CACHE_H

#include <core/Common.h>
#include <core/Environment.h>
#include <resources/ArchiveFile.h>
#include <resources/BufferLoader.h>

namespace ros {
    class ROS_API BufferCache : public boost::noncopyable {
        public:
            virtual ~BufferCache() {}

            virtual bool init(const PropertyTree& config);
            virtual void uninit();

            ArchiveEntryPtr findEntry(const std::string& name) const;
            BufferLoaderPtr findLoader(const std::string& name) const;

            virtual BufferPtr acquireBuffer(const std::string& name) =0;
            virtual void releaseBuffer(BufferPtr buffer) =0;
            virtual U32 computeUsedSize() const =0;

            const std::string& getName() const { return name; }
            const ArchiveFileList& getArchives() const { return archives; }
            const BufferLoaderList& getLoaders() const { return loaders; }

        protected:
            BufferPtr loadBuffer(const std::string& name);

        private:
            std::string name;
            ArchiveFileList archives;
            BufferLoaderList loaders;
    };

    typedef boost::shared_ptr<BufferCache> BufferCachePtr;
    typedef Factory<BufferCache> BufferCacheFactory;
    typedef std::list<BufferCachePtr> BufferCacheList;
    typedef std::map<std::string, BufferCachePtr> BufferCacheMap;
}

#endif // ROS_BUFFER_CACHE_H

