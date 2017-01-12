/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_APPLICATION_H
#define ROS_APPLICATION_H

#include <core/Common.h>
#include <core/Environment.h>
#include <core/Factory.h>
#include <application/Events.h>
#include <application/Window.h>
#include <application/View.h>
#include <graphics/Shader.h>
#include <graphics/Program.h>
#include <graphics/ShaderManager.h>
#include <graphics/ProgramManager.h>

namespace ros {
    class Application;
    typedef boost::shared_ptr<Application> ApplicationPtr;
    typedef Factory<Application> ApplicationFactory;

    class ROS_API Application : public boost::noncopyable {
        public:
            static ApplicationPtr initInstance(const PropertyTree& config);
            static ApplicationPtr getInstance() { return application; }

            Application();
            virtual ~Application() {}

            virtual bool init(const PropertyTree& config);
            virtual void uninit();
            virtual int run();

            WindowPtr getWindow() const { return window; }
            ShaderManager& getShaderManager() { return shaderManager; }
            ProgramManager& getProgramManager() { return programManager; }

            virtual float getTicks() const =0;

            virtual WindowPtr createWindow() =0;
            virtual ShaderPtr createShader() =0;
            virtual ProgramPtr createProgram() =0;

        protected:
            typedef std::list<ViewPtr> ViewList;
            ViewList views;

            virtual bool translateEvent() =0;

            virtual void onQuitEvent();
            virtual void onKeyboardPressEvent(const KeyboardPressEvent& event);
            virtual void onMouseMotionEvent(const MouseMotionEvent& event);
            virtual void onMousePressEvent(const MousePressEvent& event);
            virtual void onUpdateEvent(float duration);
            virtual void onRenderEvent();

        private:
            static ApplicationFactory factory;
            static ApplicationPtr application;
            WindowPtr window;
            ShaderManager shaderManager;
            ProgramManager programManager;
            float framesPerSecond;
            float maxAccumulatedTicks;
            bool quitOnEscape;
            bool quitRequested;

            bool initWindow(const PropertyTree& config);
    };
}

#endif // ROS_APPLICATION_H

