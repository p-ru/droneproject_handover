#ifndef TRAJGEN__VISIBILITY_CONTROL_H_
#define TRAJGEN__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define TRAJGEN_EXPORT __attribute__ ((dllexport))
    #define TRAJGEN_IMPORT __attribute__ ((dllimport))
  #else
    #define TRAJGEN_EXPORT __declspec(dllexport)
    #define TRAJGEN_IMPORT __declspec(dllimport)
  #endif
  #ifdef TRAJGEN_BUILDING_LIBRARY
    #define TRAJGEN_PUBLIC TRAJGEN_EXPORT
  #else
    #define TRAJGEN_PUBLIC TRAJGEN_IMPORT
  #endif
  #define TRAJGEN_PUBLIC_TYPE TRAJGEN_PUBLIC
  #define TRAJGEN_LOCAL
#else
  #define TRAJGEN_EXPORT __attribute__ ((visibility("default")))
  #define TRAJGEN_IMPORT
  #if __GNUC__ >= 4
    #define TRAJGEN_PUBLIC __attribute__ ((visibility("default")))
    #define TRAJGEN_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define TRAJGEN_PUBLIC
    #define TRAJGEN_LOCAL
  #endif
  #define TRAJGEN_PUBLIC_TYPE
#endif
#endif  // TRAJGEN__VISIBILITY_CONTROL_H_
// Generated 24-Nov-2025 01:30:29
// Copyright 2019-2020 The MathWorks, Inc.
