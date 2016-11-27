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
#include <application/Window.h>

namespace ros {
    class Application;
    typedef boost::shared_ptr<Application> ApplicationPtr;
    typedef Factory<std::string, Application> ApplicationFactory;

    class ROS_API Application : public boost::noncopyable {
        public:
            static ApplicationPtr create(const PropertyTree& config);
            static ApplicationPtr getInstance() { return application; }

            virtual ~Application() {}

            virtual bool init(const PropertyTree& config) =0;
            virtual void uninit() =0;
            virtual int run() =0;
            virtual WindowPtr getWindow() const =0;

        private:
            static ApplicationFactory factory;
            static ApplicationPtr application;
    };
}

#endif // ROS_APPLICATION_H

