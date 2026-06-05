#pragma once

#include <cstdio>
#include <iostream>

namespace vision_debug
{
struct NullStream
{
    template <typename T>
    NullStream &operator<<(const T &)
    {
        return *this;
    }

    NullStream &operator<<(std::ostream &(*)(std::ostream &))
    {
        return *this;
    }
};

inline NullStream null_stream;
}

#ifndef VISION_ENABLE_LOG
#define VISION_ENABLE_LOG 0
#endif

#ifndef VISION_ENABLE_SERIAL_LOG
#define VISION_ENABLE_SERIAL_LOG 0
#endif

#ifndef VISION_ENABLE_PREDICTOR_LOG
#define VISION_ENABLE_PREDICTOR_LOG 0
#endif

#ifndef VISION_ENABLE_DETECTOR_LOG
#define VISION_ENABLE_DETECTOR_LOG 0
#endif

#ifndef VISION_ENABLE_DEBUG_UI
#define VISION_ENABLE_DEBUG_UI 0
#endif

#ifndef VISION_ENABLE_CAMERA_TUNING_UI
#define VISION_ENABLE_CAMERA_TUNING_UI 0
#endif

#ifndef VISION_ENABLE_DETECTOR_DRAW
#define VISION_ENABLE_DETECTOR_DRAW 0
#endif

#if VISION_ENABLE_LOG
#define VISION_LOG_STREAM std::cout
#define VISION_LOG(expr) \
    do                   \
    {                    \
        std::cout << expr; \
    } while (0)
#else
#define VISION_LOG_STREAM ::vision_debug::null_stream
#define VISION_LOG(expr) \
    do                   \
    {                    \
    } while (0)
#endif

#if VISION_ENABLE_SERIAL_LOG
#define VISION_SERIAL_STREAM std::cout
#define VISION_SERIAL_LOG(expr) \
    do                          \
    {                           \
        std::cout << expr;      \
    } while (0)
#else
#define VISION_SERIAL_STREAM ::vision_debug::null_stream
#define VISION_SERIAL_LOG(expr) \
    do                          \
    {                           \
    } while (0)
#endif

#if VISION_ENABLE_DETECTOR_LOG
#define VISION_DETECTOR_STREAM std::cout
#define VISION_DETECTOR_LOG(expr) \
    do                            \
    {                             \
        std::cout << expr;        \
    } while (0)
#define VISION_DETECTOR_PRINTF(...) std::printf(__VA_ARGS__)
#else
#define VISION_DETECTOR_STREAM ::vision_debug::null_stream
#define VISION_DETECTOR_LOG(expr) \
    do                            \
    {                             \
    } while (0)
#define VISION_DETECTOR_PRINTF(...) \
    do                              \
    {                               \
    } while (0)
#endif

#if VISION_ENABLE_PREDICTOR_LOG
#define VISION_PREDICTOR_STREAM std::cout
#define VISION_PREDICTOR_LOG(expr) \
    do                             \
    {                              \
        std::cout << expr;         \
    } while (0)
#else
#define VISION_PREDICTOR_STREAM ::vision_debug::null_stream
#define VISION_PREDICTOR_LOG(expr) \
    do                             \
    {                              \
    } while (0)
#endif
