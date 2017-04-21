/*
 * Copyright (c) 2017 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef SDL_IMAGE_LOADER_H
#define SDL_IMAGE_LOADER_H

#include <core/Common.h>
#include <resources/ResourceLoader.h>

namespace ros {
    class SDLImageLoader : public ResourceLoader {
        public:
            virtual bool isLoadable(const std::string& resourceName) const;
            virtual ResourcePtr load(const std::string& resourceName);

        private:
            boost::regex loadable;
    };
}

#endif // SDL_IMAGE_LOADER_H

