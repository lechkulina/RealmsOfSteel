/*
 * Copyright (c) 2016 Lech Kulina
 *
 * This file is part of the Realms Of Steel.
 * For conditions of distribution and use, see copyright details in the LICENSE file.
 */
#ifndef ROS_ENV_H
#define ROS_ENV_H

#if defined(i386) || defined(__i386) || defined(__i386__) || defined(__IA32__) || \
    defined(_M_I86) || defined(_M_IX86) || defined(__X86__) || defined(_X86_) || \
    defined(__THW_INTEL__) || defined(__I86__) || defined(__INTEL__)
    #define ROS_ARCH_IA32
    #define ROS_ARCH_X86_X64
#endif
#if defined(__ia64__) || defined(_IA64) || defined(__IA64__) || defined(_M_IA64) || \
    defined(__itanium__)
    #define ROS_ARCH_IA64
#endif
#if defined(__amd64__) || defined(__amd64) || defined(__x86_64__) || defined(__x86_64) || \
    defined(_M_X64) || defined(_M_AMD64)
    #define ROS_ARCH_AMD64
    #define ROS_ARCH_X86_X64
#endif
#if !defined(ROS_ARCH_IA32) && !defined(ROS_ARCH_IA64) && !defined(ROS_ARCH_AMD64)
    #error "Failed to detect the architecture!"
#endif

#if defined(__linux__) || defined(linux) || defined(__linux)
    #define ROS_OS_LINUX
#endif
#if defined(_WIN16) || defined(_WIN32) || defined(_WIN64) || defined(__WIN32__) || defined(__TOS_WIN__) || defined(__WINDOWS__)
    #define ROS_OS_WINOWS
#endif
#if !defined(ROS_OS_LINUX) && !defined(ROS_OS_WINOWS)
    #error "Failed to detect the operating system!"
#endif

#if defined(__CYGWIN__)
    #define ROS_VARIANT_CYGWIN
#endif
#if defined(__MINGW32__) || defined(__MINGW64__)
    #define ROS_VARIANT_MINGW
#endif

#if defined(__GNUC__)
    #define ROS_CC_GCC
    #if __GNUC__ >= 4
        #define ROS_FEATURE_VISIBILITY_ATTRIBUTE
    #endif
#endif
#if defined(_MSC_VER)
    #define ROS_CC_MSVC
#endif
#if !defined(ROS_CC_GCC) && !defined(ROS_CC_MSVC)
    #error "Failed to detect the compiler!"
#endif

#ifdef ROS_OS_WINOWS
    // MSVC or GCC under Windows
    #define ROS_IMPORT __declspec(dllimport)
    #define ROS_EXPORT __declspec(dllexport)
    #define ROS_LOCAL
#elif defined(ROS_FEATURE_VISIBILITY_ATTRIBUTE)
    // Linux
    #define ROS_IMPORT __attribute__ ((visibility ("default")))
    #define ROS_EXPORT __attribute__ ((visibility ("default")))
    #define ROS_LOCAL __attribute__ ((visibility ("hidden")))
#else
    #define ROS_IMPORT
    #define ROS_EXPORT
    #define ROS_LOCAL
#endif

#ifdef ROS_LINKAGE_SHARED
    #define ROS_API ROS_EXPORT
#elif defined(ROS_STATIC_LINKAGE)
    #define ROS_API
#else
    #define ROS_API ROS_IMPORT
#endif

#endif // ROS_ENV_H

