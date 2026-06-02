#pragma once

#if defined _WIN32 || defined __CYGWIN__
#  define Impedance_DLLIMPORT __declspec(dllimport)
#  define Impedance_DLLEXPORT __declspec(dllexport)
#  define Impedance_DLLLOCAL
#else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#    define Impedance_DLLIMPORT __attribute__((visibility("default")))
#    define Impedance_DLLEXPORT __attribute__((visibility("default")))
#    define Impedance_DLLLOCAL __attribute__((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#    define Impedance_DLLIMPORT
#    define Impedance_DLLEXPORT
#    define Impedance_DLLLOCAL
#  endif // __GNUC__ >= 4
#endif // defined _WIN32 || defined __CYGWIN__

#ifdef Impedance_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define Impedance_DLLAPI
#  define Impedance_LOCAL
#else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef Impedance_EXPORTS
#    define Impedance_DLLAPI Impedance_DLLEXPORT
#  else
#    define Impedance_DLLAPI Impedance_DLLIMPORT
#  endif // Impedance_EXPORTS
#  define Impedance_LOCAL Impedance_DLLLOCAL
#endif // Impedance_STATIC