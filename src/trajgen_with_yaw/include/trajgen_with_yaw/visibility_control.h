#ifndef TRAJGEN_WITH_YAW__VISIBILITY_CONTROL_H_
#define TRAJGEN_WITH_YAW__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define TRAJGEN_WITH_YAW_EXPORT __attribute__ ((dllexport))
    #define TRAJGEN_WITH_YAW_IMPORT __attribute__ ((dllimport))
  #else
    #define TRAJGEN_WITH_YAW_EXPORT __declspec(dllexport)
    #define TRAJGEN_WITH_YAW_IMPORT __declspec(dllimport)
  #endif
  #ifdef TRAJGEN_WITH_YAW_BUILDING_LIBRARY
    #define TRAJGEN_WITH_YAW_PUBLIC TRAJGEN_WITH_YAW_EXPORT
  #else
    #define TRAJGEN_WITH_YAW_PUBLIC TRAJGEN_WITH_YAW_IMPORT
  #endif
  #define TRAJGEN_WITH_YAW_PUBLIC_TYPE TRAJGEN_WITH_YAW_PUBLIC
  #define TRAJGEN_WITH_YAW_LOCAL
#else
  #define TRAJGEN_WITH_YAW_EXPORT __attribute__ ((visibility("default")))
  #define TRAJGEN_WITH_YAW_IMPORT
  #if __GNUC__ >= 4
    #define TRAJGEN_WITH_YAW_PUBLIC __attribute__ ((visibility("default")))
    #define TRAJGEN_WITH_YAW_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define TRAJGEN_WITH_YAW_PUBLIC
    #define TRAJGEN_WITH_YAW_LOCAL
  #endif
  #define TRAJGEN_WITH_YAW_PUBLIC_TYPE
#endif
#endif  // TRAJGEN_WITH_YAW__VISIBILITY_CONTROL_H_
// Generated 26-Nov-2025 20:40:32
// Copyright 2019-2020 The MathWorks, Inc.
