/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_RAW_FILE_H
#define ROS_RAW_FILE_H

#include <boost/endian/conversion.hpp>
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

            template<class Type>
            inline bool read(Type& dst) const {
                return this->read(&dst, sizeof(dst));
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

            bool flush();
            bool write(const void* src, U32 size);

            template<class Type>
            inline bool write(const Type& src) {
                return this->write(&src, sizeof(src));
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
            FILE* stream;
            std::string path;
            FileType type;
            FileOpenMode openMode;
            FileBufferMode bufferMode;
            U32 bufferSize;

            bool openStream(const char* path, FileType type, FileOpenMode mode);
            bool setBuffering(FileBufferMode mode, U32 size);
    };

    typedef boost::shared_ptr<RawFile> RawFilePtr;
}

#endif // ROS_RAW_FILE_H

