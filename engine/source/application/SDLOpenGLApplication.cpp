/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <boost/bind.hpp>
#include <boost/range.hpp>
#include <application/Logger.h>
#include "SDLOpenGLApplication.h"
#include "SDLOpenGLWindow.h"

namespace {
    const struct KeyboardButtonMapping {
        SDL_Scancode code;
        ros::KeyboardButton button;
    } keyboardButtonMappings[] = {
        {SDL_SCANCODE_A, ros::KeyboardButton_A},
        {SDL_SCANCODE_B, ros::KeyboardButton_B},
        {SDL_SCANCODE_C, ros::KeyboardButton_C},
        {SDL_SCANCODE_D, ros::KeyboardButton_D},
        {SDL_SCANCODE_E, ros::KeyboardButton_E},
        {SDL_SCANCODE_F, ros::KeyboardButton_F},
        {SDL_SCANCODE_G, ros::KeyboardButton_G},
        {SDL_SCANCODE_H, ros::KeyboardButton_H},
        {SDL_SCANCODE_I, ros::KeyboardButton_I},
        {SDL_SCANCODE_J, ros::KeyboardButton_J},
        {SDL_SCANCODE_K, ros::KeyboardButton_K},
        {SDL_SCANCODE_L, ros::KeyboardButton_L},
        {SDL_SCANCODE_M, ros::KeyboardButton_M},
        {SDL_SCANCODE_N, ros::KeyboardButton_N},
        {SDL_SCANCODE_O, ros::KeyboardButton_O},
        {SDL_SCANCODE_P, ros::KeyboardButton_P},
        {SDL_SCANCODE_Q, ros::KeyboardButton_Q},
        {SDL_SCANCODE_R, ros::KeyboardButton_R},
        {SDL_SCANCODE_S, ros::KeyboardButton_S},
        {SDL_SCANCODE_T, ros::KeyboardButton_T},
        {SDL_SCANCODE_U, ros::KeyboardButton_U},
        {SDL_SCANCODE_V, ros::KeyboardButton_V},
        {SDL_SCANCODE_W, ros::KeyboardButton_W},
        {SDL_SCANCODE_X, ros::KeyboardButton_X},
        {SDL_SCANCODE_Y, ros::KeyboardButton_Y},
        {SDL_SCANCODE_Z, ros::KeyboardButton_Z},
        {SDL_SCANCODE_1, ros::KeyboardButton_1},
        {SDL_SCANCODE_2, ros::KeyboardButton_2},
        {SDL_SCANCODE_3, ros::KeyboardButton_3},
        {SDL_SCANCODE_4, ros::KeyboardButton_4},
        {SDL_SCANCODE_5, ros::KeyboardButton_5},
        {SDL_SCANCODE_6, ros::KeyboardButton_6},
        {SDL_SCANCODE_7, ros::KeyboardButton_7},
        {SDL_SCANCODE_8, ros::KeyboardButton_8},
        {SDL_SCANCODE_9, ros::KeyboardButton_9},
        {SDL_SCANCODE_0, ros::KeyboardButton_0}
    };

    ros::KeyboardButton KeyboardButton_fromSDLScanCode(SDL_Scancode code) {
        const KeyboardButtonMapping* iter = std::find_if(boost::begin(keyboardButtonMappings), boost::end(keyboardButtonMappings),
            boost::bind(&KeyboardButtonMapping::code, _1) == code);
        if (iter != boost::end(keyboardButtonMappings)) {
            return iter->button;
        }
        return ros::KeyboardButton_Unknown;
    }

    ros::KeyboardModifiers KeyboardModifiers_fromSDLModifiers(Uint16 modifiers) {
        if (modifiers & KMOD_LSHIFT)
            return ros::KeyboardModifiers_LeftShift;
        if (modifiers & KMOD_RSHIFT)
            return ros::KeyboardModifiers_RightShift;

        if (modifiers & KMOD_LALT)
            return ros::KeyboardModifiers_LeftAlt;
        if (modifiers & KMOD_RALT)
            return ros::KeyboardModifiers_RightAlt;

        if (modifiers & KMOD_LCTRL)
            return ros::KeyboardModifiers_LeftControl;
        if (modifiers & KMOD_RCTRL)
            return ros::KeyboardModifiers_RightControl;

        if (modifiers & KMOD_NUM)
            return ros::KeyboardModifiers_NumLock;
        if (modifiers & KMOD_CAPS)
            return ros::KeyboardModifiers_CapsLock;
        if (modifiers != KMOD_NONE)
            return ros::KeyboardModifiers_None;

        return ros::KeyboardModifiers_Unknown;
    }

    inline ros::ButtonState ButtonState_fromSDLState(Uint8 state) {
        return state == SDL_PRESSED ? ros::ButtonState_Pressed : ros::ButtonState_Released;
    }

    const struct MouseButtonMapping {
        Uint8 code;
        ros::MouseButton button;
    } mouseButtonMappings[] = {
        {SDL_BUTTON_LEFT, ros::MouseButton_Left},
        {SDL_BUTTON_MIDDLE, ros::MouseButton_Middle},
        {SDL_BUTTON_RIGHT, ros::MouseButton_Right}
    };

    ros::MouseButton MouseButton_fromSDLButton(Uint8 code) {
        const MouseButtonMapping* iter = std::find_if(boost::begin(mouseButtonMappings), boost::end(mouseButtonMappings),
            boost::bind(&MouseButtonMapping::code, _1) == code);
        if (iter != boost::end(mouseButtonMappings)) {
            return iter->button;
        }
        return ros::MouseButton_Unknown;
    }
}

ros::SDLOpenGLApplication::~SDLOpenGLApplication() {
    uninit();
}

bool ros::SDLOpenGLApplication::preInit(const PropertyTree&) {
    if (SDL_Init(SDL_INIT_VIDEO|SDL_INIT_EVENTS) != 0) {
        Logger::report(LogLevel_Error, boost::format("Failed to initialize application: %s") % SDL_GetError());
        return false;
    }

    SDL_version version;
    memset(&version, 0, sizeof(version));
    SDL_GetVersion(&version);
    Logger::report(LogLevel_Debug, boost::format("Using SDL %d.%d.%d") % (int)version.major % (int)version.minor % (int)version.patch);

    return true;
}

bool ros::SDLOpenGLApplication::postInit(const PropertyTree&) {
    return true;
}

void ros::SDLOpenGLApplication::uninit() {
    Application::uninit();
    SDL_Quit();
}

float ros::SDLOpenGLApplication::getTicks() const {
    return (float)SDL_GetTicks();
}

ros::WindowPtr ros::SDLOpenGLApplication::createWindow(const PropertyTree& config) {
    WindowPtr window = boost::make_shared<SDLOpenGLWindow>();
    if (!window || !window->init(config)) {
        return WindowPtr();
    }
    return window;
}

bool ros::SDLOpenGLApplication::translateEvent() {
    SDL_Event event;
    if (SDL_PollEvent(&event) == 0) {
        return false;
    }

    switch (event.type) {
        case SDL_QUIT: {
            onQuitEvent();
        } break;

        case SDL_KEYDOWN:
        case SDL_KEYUP: {
            KeyboardPressEvent ev;
            ev.button = KeyboardButton_fromSDLScanCode(event.key.keysym.scancode);
            ev.modifiers = KeyboardModifiers_fromSDLModifiers(event.key.keysym.mod);
            ev.state = ButtonState_fromSDLState(event.key.state);

            onKeyboardPressEvent(ev);
        } break;

        case SDL_MOUSEMOTION: {
            MouseMotionEvent ev;
            ev.x = (float)event.motion.x / getWindow()->getWidth();
            ev.y = (float)event.motion.y / getWindow()->getHeight();

            onMouseMotionEvent(ev);
        } break;

        case SDL_MOUSEBUTTONDOWN:
        case SDL_MOUSEBUTTONUP: {
            MousePressEvent ev;
            ev.x = (float)event.button.x / getWindow()->getWidth();
            ev.y = (float)event.button.y / getWindow()->getHeight();
            ev.button = MouseButton_fromSDLButton(event.button.button);
            ev.state = ButtonState_fromSDLState(event.button.state);

            onMousePressEvent(ev);
        } break;

        default:
            break;
    }

    return true;
}
