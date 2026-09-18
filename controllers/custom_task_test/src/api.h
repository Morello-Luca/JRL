#pragma once

#if defined _WIN32 || defined __CYGWIN__
#  define CustomTask_DLLIMPORT __declspec(dllimport)
#  define CustomTask_DLLEXPORT __declspec(dllexport)
#  define CustomTask_DLLLOCAL
#else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#    define CustomTask_DLLIMPORT __attribute__((visibility("default")))
#    define CustomTask_DLLEXPORT __attribute__((visibility("default")))
#    define CustomTask_DLLLOCAL __attribute__((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#    define CustomTask_DLLIMPORT
#    define CustomTask_DLLEXPORT
#    define CustomTask_DLLLOCAL
#  endif // __GNUC__ >= 4
#endif // defined _WIN32 || defined __CYGWIN__

#ifdef CustomTask_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define CustomTask_DLLAPI
#  define CustomTask_LOCAL
#else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef CustomTask_EXPORTS
#    define CustomTask_DLLAPI CustomTask_DLLEXPORT
#  else
#    define CustomTask_DLLAPI CustomTask_DLLIMPORT
#  endif // CustomTask_EXPORTS
#  define CustomTask_LOCAL CustomTask_DLLLOCAL
#endif // CustomTask_STATIC