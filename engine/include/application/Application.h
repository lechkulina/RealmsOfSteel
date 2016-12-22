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

namespace ros {
    class Application;
    typedef boost::shared_ptr<Application> ApplicationPtr;
    typedef Factory<std::string, Application> ApplicationFactory;

    class ROS_API Application : public boost::noncopyable {
        public:
            static ApplicationPtr create(const PropertyTree& config);
            static ApplicationPtr getInstance() { return application; }

            Application();
            virtual ~Application() {}

            virtual bool init(const PropertyTree& config);
            virtual void uninit();
            virtual int run();

            WindowPtr getWindow() const { return window; }

            virtual float getTicks() const =0;

        protected:
            typedef std::list<ViewPtr> ViewList;
            ViewList views;

            virtual bool preInit(const PropertyTree& config) =0;
            virtual bool postInit(const PropertyTree& config) =0;
            virtual WindowPtr createWindow(const PropertyTree& config) =0;

            virtual bool translateEvent() =0;

            virtual void onQuitEvent();
            virtual void onKeyboardPressEvent(const KeyboardPressEvent& event);
            virtual void onMouseMotionEvent(const MouseMotionEvent& event);
            virtual void onMousePressEvent(const MousePressEvent& event);
            virtual void onUpdateEvent(float duration);
            virtual void onRenderEvent();

        private:
            static const float DEFAULT_FRAMES_PER_SECOND;
            static const float DEFAULT_MAX_ACCUMULATED_TICKS;
            static const bool DEFAULT_QUIT_ON_ESCAPE;

            static ApplicationFactory factory;
            static ApplicationPtr application;
            WindowPtr window;
            float framesPerSecond;
            float maxAccumulatedTicks;
            bool quitOnEscape;
            bool isQuitRequested;
    };
}

#endif // ROS_APPLICATION_H

