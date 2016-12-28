/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_SDL_OPENGL_WINDOW_H
#define ROS_SDL_OPENGL_WINDOW_H

#include <SDL2/SDL.h>
#include <GL/glew.h>
#include <GL/GL.h>
#include <application/Window.h>

namespace ros {
    class ROS_API SDLOpenGLWindow : public Window {
        public:
            SDLOpenGLWindow();
            virtual ~SDLOpenGLWindow();

            virtual bool init(const PropertyTree& config);
            virtual void uninit();

            virtual bool isDoubleBuffered() const { return doubleBuffered; }
            virtual bool isResizable() const { return resizable; }
            virtual bool isFullscreen() const { return fullscreen; }
            virtual int getWidth() const { return width; }
            virtual int getHeight() const { return height; }

            virtual void swapBuffers();
            virtual void clearBuffers();

        private:
            SDL_Window* window;
            SDL_GLContext context;
            bool doubleBuffered;
            bool resizable;
            bool fullscreen;
            int width;
            int height;

            bool setAttributes(const PropertyTree& config);
            bool initWindow(const PropertyTree& config);
            bool initContext();

    };
}

#endif // ROS_SDL_OPENGL_WINDOW_H

