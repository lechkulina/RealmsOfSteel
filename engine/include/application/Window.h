/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_WINDOW_H
#define ROS_WINDOW_H

#include <core/Common.h>
#include <core/Environment.h>

namespace ros {
    class ROS_API Window : public boost::noncopyable {
        public:
            virtual ~Window() {}

            virtual bool init(const PropertyTree& config) =0;
            virtual void uninit() =0;

            virtual int getWidth() const =0;
            virtual int getHeight() const =0;
            virtual bool isDoubleBuffered() const =0;
            virtual bool isResizable() const =0;
            virtual bool isFullscreen() const =0;

            virtual void swapBuffers() =0;
            virtual void clearBuffers() =0;
    };

    typedef boost::shared_ptr<Window> WindowPtr;
}

#endif // ROS_WINDOW_H

