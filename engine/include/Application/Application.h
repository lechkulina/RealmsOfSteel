/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_APPLICATION_H
#define ROS_APPLICATION_H

#include <boost/shared_ptr.hpp>
#include <Core/PropertyTree.h>
#include <Core/Environment.h>
#include <Core/Factory.h>

namespace ros {

    class Application;
    typedef boost::shared_ptr<Application> ApplicationPtr;

    class ROS_API Application : public boost::noncopyable {
        public:
            static ApplicationPtr Create(const PropertyTree& config);

            virtual ~Application() {}

            virtual bool Init(const PropertyTree& config) =0;
            virtual int Run() =0;
            virtual void Uninit() =0;

        private:
            typedef Factory<std::string, Application> ApplicationFactory;

            static ApplicationFactory factory;
    };

}

#endif // ROS_APPLICATION_H

