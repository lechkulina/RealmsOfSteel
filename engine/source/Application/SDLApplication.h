/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_SDL_APPLICATION_H
#define ROS_SDL_APPLICATION_H

#include <SDL2/SDL.h>
#include <GL/glew.h>
#include <GL/GL.h>
#include <Application/Application.h>

namespace ros {

    class ROS_API SDLApplication : public Application {
        public:
            virtual bool Init(const PropertyTree& config);
            virtual int Run();
            virtual void Uninit();

        private:
            SDL_Window* window;
            SDL_GLContext context;
            bool hasQuit;

            void OnEvent(const SDL_Event& event);
            void OnUpdate(float);
            void OnRender();
    };

}

#endif // ROS_SDL_APPLICATION_H

