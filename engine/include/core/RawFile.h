/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_RAW_FILE_H
#define ROS_RAW_FILE_H

#include <core/File.h>

namespace ros {
    class ROS_API RawFile: public File {
        public:
            static const FileBufferMode DEFAULT_BUFFER_MODE = FileBufferMode_Default;
            static const U32 DEFAULT_BUFFER_SIZE = 1024;

            RawFile();
            RawFile(const char* path, FileType type, FileOpenMode openMode,
                    FileBufferMode bufferMode = DEFAULT_BUFFER_MODE, U32 bufferSize = DEFAULT_BUFFER_SIZE);
            virtual ~RawFile();

            bool open(const char* path, FileType type, FileOpenMode openMode,
                      FileBufferMode bufferMode = DEFAULT_BUFFER_MODE, U32 bufferSize = DEFAULT_BUFFER_SIZE);
            void close();
            FileType getType() const { return type; }
            FileOpenMode getOpenMode() const { return openMode; }
            FileBufferMode getBufferMode() const { return bufferMode; }
            U32 getBufferSize() const { return bufferSize; }

            virtual bool isOpen() const { return stream; }
            virtual const std::string& getPath() const { return path; }

            U32 getPosition() const;
            bool seek(S64 offset, FileOrigin origin) const;
            bool rewind() const;
            bool hasEnded() const;

            bool read(void* dst, U32 size) const;
            bool flush();
            bool write(const void* src, U32 size);

        private:
            FILE* stream;
            std::string path;
            FileType type;
            FileOpenMode openMode;
            FileBufferMode bufferMode;
            U32 bufferSize;

            bool openStream(const char* path, FileType type, FileOpenMode mode);
            bool setBuffering(FileBufferMode mode, U32 size);
    };
}

#endif // ROS_RAW_FILE_H

