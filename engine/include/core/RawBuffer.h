/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_RAW_BUFFER_H
#define ROS_RAW_BUFFER_H

#include <boost/endian/conversion.hpp>
#include <core/Common.h>
#include <core/Environment.h>
#include <core/Buffer.h>

namespace ros {
    class ROS_API RawBuffer: public Buffer {
        public:
            RawBuffer();
            explicit RawBuffer(U32 size);
            virtual ~RawBuffer();

            RawBuffer(const RawBuffer& src);
            RawBuffer& operator=(const RawBuffer& src);

            bool allocate(U32 size);
            bool assign(const RawBuffer& src);
            bool resize(U32 size);
            void free();
            void clear();

            virtual bool isNull() const { return !data; }
            virtual U32 getSize() const { return size; }

            template<class Type>
            inline Type* at(U32 offset = 0) {
                if (!data || offset >= size) {
                    return ROS_NULL;
                }
                return reinterpret_cast<Type*>(data + offset);
            }

            template<class Type>
            inline const Type* at(U32 offset = 0) const {
                if (!data || offset >= size) {
                    return ROS_NULL;
                }
                return reinterpret_cast<const Type*>(data + offset);
            }

            U32 getPosition() const { return position; }
            bool seek(S64 offset, BufferOrigin origin) const;
            bool rewind() const;
            bool hasEnded() const;

            bool read(void* dst, U32 size) const;

            template<class Type>
            inline bool read(Type& dst) const {
                return this->read(&dst, sizeof(Type));
            }

            template<class Type>
            bool readLittle(Type& dst) const {
                if (!read<Type>(dst)) {
                    return false;
                }
                boost::endian::little_to_native_inplace(dst);
                return true;
            }

            template<class Type>
            bool readBig(Type& dst) const {
                if (!read(&dst, sizeof(dst))) {
                    return false;
                }
                boost::endian::big_to_native_inplace(dst);
                return true;
            }

            bool write(const void* src, U32 size);

            template<class Type>
            inline bool write(const Type& src) {
                return this->write(&src, sizeof(Type));
            }

            template<class Type>
            inline bool writeLittle(const Type& src) {
                return write(boost::endian::native_to_little(src));
            }

            template<class Type>
            inline bool writeBig(const Type& src) {
                return write(boost::endian::native_to_big(src));
            }

        private:
            U8* data;
            U32 size;
            mutable S64 position;
    };

    typedef boost::shared_ptr<RawBuffer> RawBufferPtr;
}

#endif // ROS_RAW_BUFFER_H

