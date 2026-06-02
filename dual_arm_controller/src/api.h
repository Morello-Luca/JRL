#pragma once

#if defined _WIN32 || defined __CYGWIN__
#  define DualArmControl_DLLIMPORT __declspec(dllimport)
#  define DualArmControl_DLLEXPORT __declspec(dllexport)
#  define DualArmControl_DLLLOCAL
#else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#    define DualArmControl_DLLIMPORT __attribute__((visibility("default")))
#    define DualArmControl_DLLEXPORT __attribute__((visibility("default")))
#    define DualArmControl_DLLLOCAL __attribute__((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#    define DualArmControl_DLLIMPORT
#    define DualArmControl_DLLEXPORT
#    define DualArmControl_DLLLOCAL
#  endif // __GNUC__ >= 4
#endif // defined _WIN32 || defined __CYGWIN__

#ifdef DualArmControl_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define DualArmControl_DLLAPI
#  define DualArmControl_LOCAL
#else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef DualArmControl_EXPORTS
#    define DualArmControl_DLLAPI DualArmControl_DLLEXPORT
#  else
#    define DualArmControl_DLLAPI DualArmControl_DLLIMPORT
#  endif // DualArmControl_EXPORTS
#  define DualArmControl_LOCAL DualArmControl_DLLLOCAL
#endif // DualArmControl_STATIC