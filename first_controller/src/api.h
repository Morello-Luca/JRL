#pragma once

#if defined _WIN32 || defined __CYGWIN__
#  define Ur5Manipulation_DLLIMPORT __declspec(dllimport)
#  define Ur5Manipulation_DLLEXPORT __declspec(dllexport)
#  define Ur5Manipulation_DLLLOCAL
#else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#    define Ur5Manipulation_DLLIMPORT __attribute__((visibility("default")))
#    define Ur5Manipulation_DLLEXPORT __attribute__((visibility("default")))
#    define Ur5Manipulation_DLLLOCAL __attribute__((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#    define Ur5Manipulation_DLLIMPORT
#    define Ur5Manipulation_DLLEXPORT
#    define Ur5Manipulation_DLLLOCAL
#  endif // __GNUC__ >= 4
#endif // defined _WIN32 || defined __CYGWIN__

#ifdef Ur5Manipulation_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define Ur5Manipulation_DLLAPI
#  define Ur5Manipulation_LOCAL
#else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef Ur5Manipulation_EXPORTS
#    define Ur5Manipulation_DLLAPI Ur5Manipulation_DLLEXPORT
#  else
#    define Ur5Manipulation_DLLAPI Ur5Manipulation_DLLIMPORT
#  endif // Ur5Manipulation_EXPORTS
#  define Ur5Manipulation_LOCAL Ur5Manipulation_DLLLOCAL
#endif // Ur5Manipulation_STATIC