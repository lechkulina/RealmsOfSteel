/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_APPLICATION_H
#define ROS_APPLICATION_H

#include <boost/shared_ptr.hpp>
#include <boost/noncopyable.hpp>
#include <Core/PropertyTree.h>
#include <Core/Environment.h>
#include <Core/Factory.h>
#include <Application/Window.h>

namespace ros {

    class Application;
    typedef boost::shared_ptr<Application> ApplicationPtr;
    typedef Factory<String, Application> ApplicationFactory;

    class ROS_API Application : public boost::noncopyable {
        public:
            static ApplicationPtr Create(const PropertyTree& config);
            static ApplicationPtr GetInstance() { return application; }

            virtual ~Application() {}

            virtual bool Init(const PropertyTree& config);
            virtual int Run() =0;
            virtual void Uninit() =0;

            WindowPtr GetWindow() const { return window; }

        protected:
            WindowPtr window;

        private:
            static ApplicationFactory factory;
            static ApplicationPtr application;
    };

}

#endif // ROS_APPLICATION_H

