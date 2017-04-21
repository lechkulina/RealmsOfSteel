/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <core/RawBuffer.h>

ros::RawBuffer::RawBuffer()
    : data(ROS_NULL)
    , size(0)
    , position(0) {
}

ros::RawBuffer::RawBuffer(U32 size)
    : data(ROS_NULL)
    , size(0)
    , position(0) {
    allocate(size);
}

ros::RawBuffer::~RawBuffer() {
    free();
}

ros::RawBuffer::RawBuffer(const RawBuffer& src)
    : data(ROS_NULL)
    , size(0)
    , position(0) {
    assign(src);
}

ros::RawBuffer& ros::RawBuffer::operator=(const RawBuffer& src) {
    assign(src);
    return *this;
}

bool ros::RawBuffer::allocate(U32 size) {
    free();
    if (size == 0) {
        return false;
    }

    data = new (std::nothrow) U8[size];
    if (!data) {
        return false;
    }
    memset(data, 0, size);
    this->size = size;

    return true;
}

void ros::RawBuffer::free() {
    delete []data;
    data = ROS_NULL;
    size = 0;
    position = 0;
}

bool ros::RawBuffer::assign(const RawBuffer& src) {
    if (this == &src) {
        return true;
    }

    if (src.isNull() || !allocate(src.getSize())) {
        return false;
    }
    memcpy(data, src.at<U8>(), src.getSize());

    return true;
}

bool ros::RawBuffer::resize(U32 size) {
    if (!this->data) {
        return allocate(size);
    }

    if (size == 0) {
        return false;
    }

    U8* data = new (std::nothrow) U8[size];
    if (!data) {
        return false;
    }
    memset(data, 0, size);
    memcpy(data, this->data, std::min(size, this->size));

    free();
    this->data = data;
    this->size = size;

    return true;
}

bool ros::RawBuffer::clear() {
    if (data) {
        memset(data, 0, size);
        return true;
    }
    return false;
}

bool ros::RawBuffer::seek(S64 offset, BufferOrigin origin) const {
    if (!data) {
        return false;
    }
    if (offset == 0 && origin == BufferOrigin_Current) {
        return true;
    }

    switch(origin) {
        case BufferOrigin_Begin:
            if (offset >= static_cast<S64>(size)) {
                return false;
            }
            position = offset;
            break;
        case BufferOrigin_Current:
            if (position + offset > static_cast<S64>(size) || position + offset < 0) {
                return false;
            }
            position += offset;
            break;
        case BufferOrigin_End:
            if (offset >= 0 || static_cast<S64>(size) + offset < 0) {
                return false;
            }
            position = size + offset;
            break;
    }

    return true;
}

bool ros::RawBuffer::rewind() const {
    if (!data) {
        return false;
    }
    position = 0;
    return true;
}

bool ros::RawBuffer::hasEnded() const {
    return position >= static_cast<S64>(size);
}

bool ros::RawBuffer::read(void* dst, U32 size) const {
    if (!data || position + size > this->size) {
        return false;
    }
    if (size == 0) {
        return true;
    }

    memcpy(dst, data + position, size);
    position += size;

    return true;
}

bool ros::RawBuffer::write(const void* src, U32 size) {
    if (!data || position + size > this->size)
        return false;
    if (size == 0) {
        return true;
    }

    memcpy(data + position, src, size);
    position += size;

    return true;
}
