#pragma once

#if defined _WIN32 || defined __CYGWIN__
#  define BimanualControl_DLLIMPORT __declspec(dllimport)
#  define BimanualControl_DLLEXPORT __declspec(dllexport)
#  define BimanualControl_DLLLOCAL
#else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#    define BimanualControl_DLLIMPORT __attribute__((visibility("default")))
#    define BimanualControl_DLLEXPORT __attribute__((visibility("default")))
#    define BimanualControl_DLLLOCAL __attribute__((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#    define BimanualControl_DLLIMPORT
#    define BimanualControl_DLLEXPORT
#    define BimanualControl_DLLLOCAL
#  endif // __GNUC__ >= 4
#endif // defined _WIN32 || defined __CYGWIN__

#ifdef BimanualControl_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define BimanualControl_DLLAPI
#  define BimanualControl_LOCAL
#else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef BimanualControl_EXPORTS
#    define BimanualControl_DLLAPI BimanualControl_DLLEXPORT
#  else
#    define BimanualControl_DLLAPI BimanualControl_DLLIMPORT
#  endif // BimanualControl_EXPORTS
#  define BimanualControl_LOCAL BimanualControl_DLLLOCAL
#endif // BimanualControl_STATIC