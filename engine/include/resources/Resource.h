/*
 * Copyright (c) 2017 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_RESOURCE_H
#define ROS_RESOURCE_H

#include <core/Common.h>
#include <core/Environment.h>
#include <application/Logger.h>
#include <resources/ResourceCache.h>

namespace ros {
    template<class DerivedBuffer>
    class Resource {
        public:
            typedef boost::shared_ptr<DerivedBuffer> DerivedBufferPtr;

            Resource() { }
            explicit Resource(const std::string& name) {
                acquire(name);
            }

            virtual ~Resource() {
                release();
            }

            bool acquire(const std::string& name) {
                release();

                BufferPtr buffer = ResourceCache::getInstance()->acquireBuffer(name);
                if (!buffer) {
                    return false;
                }

                DerivedBufferPtr cast = boost::dynamic_pointer_cast<DerivedBuffer>(buffer);
                if (!cast) {
                    Logger::report(LogLevel_Critical, boost::format("Invalid buffer requested for resource %s") % name);
                    return false;
                }

                this->buffer = cast;
                return true;
            }

            void release() {
                if (buffer && buffer.unique()) {
                    ResourceCache::getInstance()->releaseBuffer(buffer);
                    buffer.reset();
                }
            }

            inline bool isNull() const { return !buffer; }
            inline operator bool() const { return buffer; }

            inline DerivedBuffer& operator*() { return *buffer; }
            inline const DerivedBuffer& operator*() const { return *buffer; }
            inline DerivedBuffer* operator->() { return buffer.get(); }
            inline const DerivedBuffer* operator->() const { return buffer.get(); }

        private:
            DerivedBufferPtr buffer;
    };

    typedef Resource<RawBuffer> RawResource;
}

#endif // ROS_RESOURCE_H

