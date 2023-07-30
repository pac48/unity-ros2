#pragma once

#include "Eigen/Core"
#include <optional>


#if defined(_MSC_VER)
#define MY_LIB_API __declspec(dllexport) // Microsoft
#elif defined(__GNUC__)
#define MY_LIB_API __attribute__((visibility("default"))) // GCC
#else
#define MY_LIB_API // Most compilers export all the symbols by default. We hope for the best here.
#pragma warning Unknown dynamic link import/export semantics.
#endif

extern "C" {
MY_LIB_API std::intptr_t Init();
MY_LIB_API void Receive(std::intptr_t handle, char *type, char *topic, void *output);
MY_LIB_API void Publish(std::intptr_t handle, char *type, char *topic, void *input);
MY_LIB_API void Destroy(std::intptr_t handle);
}