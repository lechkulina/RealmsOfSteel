/*
 * Copyright (c) 2017 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_ASSIMP_IOSTREAM_H
#define ROS_ASSIMP_IOSTREAM_H

#include <core/RawBuffer.h>
#include <assimp/IOStream.hpp>

namespace ros {
    class AssimpIOStream: public Assimp::IOStream {
        public:
            explicit AssimpIOStream(RawBufferPtr buffer);

            void setBuffer(RawBufferPtr buffer);
            RawBufferPtr getBuffer() const { return buffer; }

            virtual size_t Read(void* dst, size_t size, size_t count);
            virtual size_t Write(const void* src, size_t size, size_t count);
            virtual aiReturn Seek(size_t offset, aiOrigin origin);
            virtual size_t Tell() const;
            virtual size_t FileSize() const ;
            virtual void Flush();

        private:
            RawBufferPtr buffer;
    };
}

#endif // ROS_ASSIMP_IOSTREAM_H

