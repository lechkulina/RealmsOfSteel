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
#include <application/View.h>

namespace ros {
    enum ButtonState {
        ButtonState_Pressed,
        ButtonState_Released
    };

    enum KeyboardButton {
        KeyboardButton_Unknown,
        KeyboardButton_A,
        KeyboardButton_B,
        KeyboardButton_C,
        KeyboardButton_D,
        KeyboardButton_E,
        KeyboardButton_F,
        KeyboardButton_G,
        KeyboardButton_H,
        KeyboardButton_I,
        KeyboardButton_J,
        KeyboardButton_K,
        KeyboardButton_L,
        KeyboardButton_M,
        KeyboardButton_N,
        KeyboardButton_O,
        KeyboardButton_P,
        KeyboardButton_Q,
        KeyboardButton_R,
        KeyboardButton_S,
        KeyboardButton_T,
        KeyboardButton_U,
        KeyboardButton_V,
        KeyboardButton_W,
        KeyboardButton_X,
        KeyboardButton_Y,
        KeyboardButton_Z,
        KeyboardButton_1,
        KeyboardButton_2,
        KeyboardButton_3,
        KeyboardButton_4,
        KeyboardButton_5,
        KeyboardButton_6,
        KeyboardButton_7,
        KeyboardButton_8,
        KeyboardButton_9,
        KeyboardButton_0,
        KeyboardButton_Return,
        KeyboardButton_Escape,
        KeyboardButton_Backspace,
        KeyboardButton_Tab,
        KeyboardButton_Space,
        KeyboardButton_Minus,
        KeyboardButton_Equals,
        KeyboardButton_LeftBracket,
        KeyboardButton_RightBracket,
        KeyboardButton_BackSlash,
        KeyboardButton_Semicolon,
        KeyboardButton_Apostrophe,
        KeyboardButton_Comma,
        KeyboardButton_Period,
        KeyboardButton_Slash,
        KeyboardButton_CapsLock,
        KeyboardButton_F1,
        KeyboardButton_F2,
        KeyboardButton_F3,
        KeyboardButton_F4,
        KeyboardButton_F5,
        KeyboardButton_F6,
        KeyboardButton_F7,
        KeyboardButton_F8,
        KeyboardButton_F9,
        KeyboardButton_F10,
        KeyboardButton_F11,
        KeyboardButton_F12,
        KeyboardButton_PrintScreen,
        KeyboardButton_ScrollLock,
        KeyboardButton_Pause,
        KeyboardButton_Insert,
        KeyboardButton_Home,
        KeyboardButton_PageUp,
        KeyboardButton_Delete,
        KeyboardButton_End,
        KeyboardButton_PageDown,
        KeyboardButton_RightArrow,
        KeyboardButton_LeftArrow,
        KeyboardButton_DownArrow,
        KeyboardButton_UpArrow
    };

    typedef int KeyboardModifiers;
    const KeyboardModifiers KeyboardModifiers_None = 0;
    const KeyboardModifiers KeyboardModifiers_LeftControl = 1 << 1;
    const KeyboardModifiers KeyboardModifiers_RightControl = 1 << 2;
    const KeyboardModifiers KeyboardModifiers_Control = KeyboardModifiers_LeftControl |KeyboardModifiers_RightControl;
    const KeyboardModifiers KeyboardModifiers_LeftShift = 1 << 3;
    const KeyboardModifiers KeyboardModifiers_RightShift = 1 << 4;
    const KeyboardModifiers KeyboardModifiers_Shift = KeyboardModifiers_LeftShift | KeyboardModifiers_RightShift;
    const KeyboardModifiers KeyboardModifiers_LeftAlt = 1 << 5;
    const KeyboardModifiers KeyboardModifiers_RightAlt = 1 << 6;
    const KeyboardModifiers KeyboardModifiers_Alt = KeyboardModifiers_LeftAlt | KeyboardModifiers_RightAlt;
    const KeyboardModifiers KeyboardModifiers_NumLock = 1 << 7;
    const KeyboardModifiers KeyboardModifiers_CapsLock = 1 << 8;
    const KeyboardModifiers KeyboardModifiers_Unknown = 1 << 9;

    struct KeyboardPressEvent {
        KeyboardButton button;
        KeyboardModifiers modifiers;
        ButtonState state;
    };

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

        protected:
            typedef std::list<ViewPtr> ViewList;
            ViewList views;

            void onKeyboardPressEvent(const KeyboardPressEvent& event);

        private:
            static ApplicationFactory factory;
            static ApplicationPtr application;
    };
}

#endif // ROS_APPLICATION_H

