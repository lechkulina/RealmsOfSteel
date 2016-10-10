/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_WINDOW_H
#define ROS_WINDOW_H

#include <boost/shared_ptr.hpp>
#include <boost/noncopyable.hpp>
#include <Core/PropertyTree.h>
#include <Core/Environment.h>
#include <Core/Factory.h>

namespace ros {

    class Window;
    typedef boost::shared_ptr<Window> WindowPtr;
    typedef Factory<String, Window> WindowFactory;

    class ROS_API Window : public boost::noncopyable {
        public:
            static WindowPtr Create(const PropertyTree& config);

            virtual ~Window() {}

            virtual bool Init(const PropertyTree& config) =0;
            virtual void Uninit() =0;
            virtual void SwapBuffers() =0;
            virtual void ClearBuffers() =0;

        private:
            static WindowFactory factory;
    };

}

#endif // ROS_WINDOW_H

