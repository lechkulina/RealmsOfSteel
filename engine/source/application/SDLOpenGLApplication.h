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
            virtual ~SDLOpenGLApplication();

            virtual bool init(const PropertyTree& config);
            virtual void uninit();

            virtual float getTicks() const;

            virtual WindowPtr createWindow();
            virtual ShaderPtr createShader();
            virtual ProgramPtr createProgram();
            virtual ImagePtr createImage();

        protected:
            virtual bool translateEvent();
    };
}

#endif // ROS_SDL_OPENGL_APPLICATION_H

