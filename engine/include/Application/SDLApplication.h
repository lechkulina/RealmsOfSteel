/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_SDL_APPLICATION_H
#define ROS_SDL_APPLICATION_H

#include <Core/Environment.h>

namespace ros {

    class ROS_API SDLApplication {
        public:
            void init();
            void run();
            void quit();
    };

}


#endif // ROS_SDL_APPLICATION_H

