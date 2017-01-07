/*
 * Copyright (c) 2017 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_RESOURCE_CACHE_H
#define ROS_RESOURCE_CACHE_H

#include <core/Common.h>
#include <core/Environment.h>
#include <resources/ArchiveFile.h>
#include <resources/ResourceLoader.h>

namespace ros {
    class ROS_API ResourceCache : public boost::noncopyable {
        public:
            virtual ~ResourceCache() {}

            virtual bool init(const PropertyTree& config);
            virtual void uninit();

            virtual BufferPtr acquireBuffer(const std::string& name) =0;
            virtual void releaseBuffer(BufferPtr buffer) =0;

            const std::string& getName() const { return name; }
            ArchiveFileList& getArchives() { return archives; }
            ResourceLoaderList& getLoaders() { return loaders; }

        protected:
            bool initArchives(const PropertyTree& config);
            bool initLoaders(const PropertyTree& config);
            BufferPtr loadBuffer(const std::string& name);

        private:
            std::string name;
            ArchiveFileList archives;
            ResourceLoaderList loaders;

            ArchiveEntryPtr findEntry(const std::string& name);
            ResourceLoaderPtr findLoader(const std::string& name);
    };
}

#endif // ROS_RESOURCE_CACHE_H

