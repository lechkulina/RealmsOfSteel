/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_OPENGL_WINDOW_H
#define ROS_OPENGL_WINDOW_H

#include <SDL2/SDL.h>
#include <GL/glew.h>
#include <GL/GL.h>
#include <Application/Window.h>

namespace ros {

    class ROS_API OpenGLWindow : public Window {
        public:
            OpenGLWindow();

            virtual bool Init(const PropertyTree& config);
            virtual void Uninit();
            virtual void SwapBuffers();
            virtual void ClearBuffers();

        private:
            SDL_Window* window;
            SDL_GLContext context;
    };

}

#endif // ROS_OPENGL_WINDOW_H

