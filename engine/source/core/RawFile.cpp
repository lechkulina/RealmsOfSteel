/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <core/RawFile.h>

ros::RawFile::RawFile()
    : stream(ROS_NULL)
    , type(FileType_Invalid)
    , openMode(FileOpenMode_Invalid)
    , bufferMode(FileBufferMode_Invalid)
    , bufferSize(0) {
}

ros::RawFile::RawFile(const char* path, FileType type, FileOpenMode openMode, FileBufferMode bufferMode, U32 bufferSize)
    : stream(ROS_NULL)
    , type(FileType_Invalid)
    , openMode(FileOpenMode_Invalid)
    , bufferMode(FileBufferMode_Invalid)
    , bufferSize(0) {
    open(path, type, openMode, bufferMode, bufferSize);
}

ros::RawFile::~RawFile() {
    close();
}

bool ros::RawFile::open(const char* path, FileType type, FileOpenMode openMode, FileBufferMode bufferMode, U32 bufferSize) {
    close();
    if (!openStream(path, type, openMode) || !setBuffering(bufferMode, bufferSize)) {
        close();
        return false;
    }
    return true;
}

void ros::RawFile::close() {
    if (stream) {
        fclose(stream);
        stream = ROS_NULL;
    }
    path.clear();
    type = FileType_Invalid;
    openMode = FileOpenMode_Invalid;
    bufferMode = FileBufferMode_Invalid;
    bufferSize = 0;
}

ros::U32 ros::RawFile::getPosition() const {
    if (!stream) {
        return 0;
    }
    long int position = ftell(stream);
    return position == -1L ? 0 : static_cast<U32>(position);
}

bool ros::RawFile::seek(S64 offset, FileOrigin origin) const {
    if (!stream) {
        return false;
    }
    if (offset == 0 && origin == FileOrigin_Current) {
        return true;
    }

    int originInt = SEEK_SET;
    if (type == FileType_Binary) {
        switch(origin) {
            case FileOrigin_Begin:
                originInt = SEEK_SET;
                break;
            case FileOrigin_Current:
                originInt = SEEK_CUR;
                break;
            case FileOrigin_End:
                originInt = SEEK_END;
                break;
            default:
                return false;
        }
    }

    return fseek(stream, offset, originInt) == 0;
}

bool ros::RawFile::rewind() const {
    if (!stream) {
        return false;
    }
    ::rewind(stream);
    return true;
}

bool ros::RawFile::hasEnded() const {
    if (!stream) {
        return false;
    }
    return feof(stream) != 0;
}

bool ros::RawFile::read(void* dst, U32 size) const {
    if (!stream) {
        return false;
    }
    if (size == 0) {
        return true;
    }
    return fread(dst, 1, size, stream) == size;
}

bool ros::RawFile::flush() {
    if (!stream) {
        return false;
    }
    return fflush(stream) != EOF;
}

bool ros::RawFile::write(const void* src, U32 size) {
    if (!stream) {
        return false;
    }
    if (size == 0) {
        return true;
    }
    return fwrite(src, 1, size, stream) == size;
}

bool ros::RawFile::openStream(const char* path, FileType type, FileOpenMode mode) {
    char modeStr[4];
    memset(modeStr, 0, sizeof(modeStr) / sizeof(modeStr[0]));

    switch(mode) {
        case FileOpenMode_Read:
            strcpy(modeStr, "r");
            break;
        case FileOpenMode_Write:
            strcpy(modeStr, "w");
            break;
        case FileOpenMode_Append:
            strcpy(modeStr, "a");
            break;
        case FileOpenMode_ReadUpdate:
            strcpy(modeStr, "r+");
            break;
        case FileOpenMode_WriteUpdate:
            strcpy(modeStr, "w+");
            break;
        case FileOpenMode_AppendUpdate:
            strcpy(modeStr, "a+");
            break;
        default:
            return false;
    }
    switch(type) {
        case FileType_Text:
            break;
        case FileType_Binary:
            strcat(modeStr, "b");
            break;
        default:
            return false;
    }

    stream = fopen(path, modeStr);
    this->path = path;
    this->type = type;
    openMode = mode;

    return stream;
}

bool ros::RawFile::setBuffering(FileBufferMode mode, U32 size) {
    if (mode != FileBufferMode_Default) {
        int modeInt = 0;
        switch(mode) {
            case FileBufferMode_Full:
                modeInt = _IOFBF;
                break;
            case FileBufferMode_Line:
                modeInt = _IOLBF;
                break;
            case FileBufferMode_None:
                modeInt = _IONBF;
                size = 0;
                break;
            default:
                return false;
        }

        if (setvbuf(stream, ROS_NULL, modeInt, size) != 0) {
            return false;
        }
    } else {
        size = 0;
    }

    bufferMode = mode;
    bufferSize = size;
    return true;
}
