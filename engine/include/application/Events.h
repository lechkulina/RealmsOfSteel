/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_EVENTS_H
#define ROS_EVENTS_H

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

    struct MouseMotionEvent {
        float x;
        float y;
    };

    enum MouseButton {
        MouseButton_Unknown,
        MouseButton_Left,
        MouseButton_Middle,
        MouseButton_Right
    };

    struct MousePressEvent {
        MouseButton button;
        ButtonState state;
        float x;
        float y;
    };
}

#endif // ROS_EVENTS_H

