/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#include <boost/bind.hpp>
#include <boost/range.hpp>
#include <application/Logger.h>
#include "SDLApplication.h"

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

ros::SDLApplication::SDLApplication()
    : hasQuit(false) {
}

ros::SDLApplication::~SDLApplication() {
    uninit();
}

bool ros::SDLApplication::init(const PropertyTree& config) {
    if (SDL_Init(SDL_INIT_VIDEO|SDL_INIT_EVENTS) != 0) {
        Logger::report(LogLevel_Error, boost::format("Failed to initialize SDL application: %s") % SDL_GetError());
        uninit();
        return false;
    }

    SDL_version version;
    memset(&version, 0, sizeof(version));
    SDL_GetVersion(&version);
    Logger::report(LogLevel_Debug, boost::format("Using SDL %d.%d.%d") % (int)version.major % (int)version.minor % (int)version.patch);

    PropertyTree::const_assoc_iterator iter = config.find("Window");
    if (iter == config.not_found()) {
        Logger::report(LogLevel_Error, boost::format("Failed to initialize SDL application: Missing window configuration"));
        uninit();
        return false;
    }

    window = Window::create(iter->second);
    if (!window) {
        uninit();
        return false;
    }

    return true;
}

void ros::SDLApplication::uninit() {
    if (window) {
        window->uninit();
    }
    SDL_Quit();
}

int ros::SDLApplication::run() {
    Logger::report(LogLevel_Trace, boost::format("Starting Realms Of Steel %s") % ROS_VERSION);

    float fps = 60.0f;
    float deltaTime = (1 / fps) * 1000;
    float maxAccumulatedTime = 50.0f;
    float accumulatedTime = 0.0f;
    float startTime = (float)SDL_GetTicks();

    while (!hasQuit) {
        SDL_Event event;
        if (SDL_PollEvent(&event) > 0) {
            onEvent(event);
            continue;
        }

        float currentTime = (float)SDL_GetTicks();
        accumulatedTime += currentTime - startTime;
        if (accumulatedTime > maxAccumulatedTime) {
            accumulatedTime = maxAccumulatedTime;
        }
        startTime = currentTime;
        while (accumulatedTime > deltaTime) {
            onUpdate(deltaTime);
            accumulatedTime -= deltaTime;
        }

        window->clearBuffers();
        onRender();
        window->swapBuffers();
    }

    return EXIT_SUCCESS;
}

void ros::SDLApplication::onEvent(const SDL_Event& event) {
    switch (event.type) {
        case SDL_QUIT: {
            hasQuit = true;
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
            ev.x = (float)event.motion.x / window->getWidth();
            ev.y = (float)event.motion.y / window->getHeight();

            onMouseMotionEvent(ev);
        } break;

        case SDL_MOUSEBUTTONDOWN:
        case SDL_MOUSEBUTTONUP: {
            MousePressEvent ev;
            ev.x = (float)event.button.x / window->getWidth();
            ev.y = (float)event.button.y / window->getHeight();
            ev.button = MouseButton_fromSDLButton(event.button.button);
            ev.state = ButtonState_fromSDLState(event.button.state);

            onMousePressEvent(ev);
        } break;

        default:
            break;
    }

}

void ros::SDLApplication::onUpdate(float) {

}

void ros::SDLApplication::onRender() {

}

