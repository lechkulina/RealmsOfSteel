/*
 * Copyright (c) 2017 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include "AssimpIOStream.h"

ros::AssimpIOStream::AssimpIOStream(RawBufferPtr buffer) {
    setBuffer(buffer);
}

void ros::AssimpIOStream::setBuffer(RawBufferPtr buffer) {
    this->buffer = buffer;
    if (buffer) {
        buffer->rewind();
    }
}

size_t ros::AssimpIOStream::Read(void* dst, size_t size, size_t count) {
    if (!buffer) {
        return 0;
    }
    const U32 before = buffer->getPosition();
    if (!buffer->read(dst, size * count)) {
        return 0;
    }
    return buffer->getPosition() - before;
}

size_t ros::AssimpIOStream::Write(const void* src, size_t size, size_t count) {
    if (!buffer) {
        return 0;
    }
    const U32 before = buffer->getPosition();
    if (!buffer->write(src, size * count)) {
        return 0;
    }
    return buffer->getPosition() - before;
}

aiReturn ros::AssimpIOStream::Seek(size_t offset, aiOrigin origin) {
    if (!buffer) {
        return aiReturn_FAILURE;
    }
    S64 bufferOffset = offset;
    BufferOrigin bufferOrigin = BufferOrigin_Begin;
    switch(origin) {
        case aiOrigin_SET:
            break;
        case aiOrigin_CUR:
            bufferOrigin = BufferOrigin_Current;
            break;
        case aiOrigin_END:
            bufferOffset = -offset;
            bufferOrigin = BufferOrigin_End;
            break;
        default:
            return aiReturn_FAILURE;
    }
    if (!buffer->seek(bufferOffset, bufferOrigin)) {
        return aiReturn_FAILURE;
    }
    return aiReturn_SUCCESS;
}

size_t ros::AssimpIOStream::Tell() const {
    if (!buffer) {
        return 0;
    }
    return buffer->getPosition();
}

size_t ros::AssimpIOStream::FileSize() const {
    if (!buffer) {
        return 0;
    }
    return buffer->getSize();
}

void ros::AssimpIOStream::Flush() {
    // empty
}
