/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_SDL_OPENGL_APPLICATION_H
#define ROS_SDL_OPENGL_APPLICATION_H

#include <SDL2/SDL.h>
#include <application/Application.h>

namespace ros {
    class ROS_API SDLOpenGLApplication : public Application {
        public:
            SDLOpenGLApplication();
            virtual ~SDLOpenGLApplication();

            virtual void uninit();
            virtual int run();

        protected:
            virtual bool preInit(const PropertyTree& config);
            virtual bool postInit(const PropertyTree& config);
            virtual WindowPtr createWindow(const PropertyTree& config);

        private:
            bool hasQuit;

            void onEvent(const SDL_Event& event);
            void onUpdate(float);
            void onRender();
    };
}

#endif // ROS_SDL_OPENGL_APPLICATION_H

