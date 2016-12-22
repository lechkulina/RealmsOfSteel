/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_VIEW_H
#define ROS_VIEW_H

#include <core/Common.h>
#include <core/Environment.h>
#include <application/Events.h>

namespace ros {
    class ROS_API View : public boost::noncopyable {
        public:
            virtual ~View() {}

            virtual bool init(const PropertyTree& config) =0;
            virtual void uninit() =0;

            virtual void onKeyboardPressEvent(const KeyboardPressEvent& event) =0;
            virtual void onMouseMotionEvent(const MouseMotionEvent& event) =0;
            virtual void onMousePressEvent(const MousePressEvent& event) =0;
    };

    typedef boost::shared_ptr<View> ViewPtr;
}

#endif // ROS_VIEW_H

