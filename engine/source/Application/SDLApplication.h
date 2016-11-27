/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_SDL_APPLICATION_H
#define ROS_SDL_APPLICATION_H

#include <SDL2/SDL.h>
#include <application/Application.h>

namespace ros {
    class ROS_API SDLApplication : public Application {
        public:
            SDLApplication();
            virtual ~SDLApplication();

            virtual bool init(const PropertyTree& config);
            virtual void uninit();
            virtual int run();
            virtual WindowPtr getWindow() const { return window; }

        private:
            WindowPtr window;
            bool hasQuit;

            void onEvent(const SDL_Event& event);
            void onUpdate(float);
            void onRender();
    };
}

#endif // ROS_SDL_APPLICATION_H

