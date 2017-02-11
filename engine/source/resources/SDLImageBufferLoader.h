/*
 * Copyright (c) 2017 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef SDL_IMAGE_BUFFER_LOADER_H
#define SDL_IMAGE_BUFFER_LOADER_H

#include <boost/regex.hpp>
#include <core/Common.h>
#include <resources/BufferLoader.h>

namespace ros {
    class SDLImageBufferLoader : public BufferLoader {
        public:
            virtual ~SDLImageBufferLoader();

            virtual bool init(const PropertyTree& config);
            virtual void uninit();

            virtual bool isLoadable(const std::string& name) const;
            virtual BufferPtr loadBuffer(RawBufferPtr src);

        private:
            boost::regex loadable;
    };
}

#endif // SDL_IMAGE_BUFFER_LOADER_H

