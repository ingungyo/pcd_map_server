#ifndef PCD_MAP_SERVER__VISIBILITY_CONTROL_HPP_
#define PCD_MAP_SERVER__VISIBILITY_CONTROL_HPP_

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define PCD_MAP_SERVER_EXPORT __attribute__ ((dllexport))
    #define PCD_MAP_SERVER_IMPORT __attribute__ ((dllimport))
  #else
    #define PCD_MAP_SERVER_EXPORT __declspec(dllexport)
    #define PCD_MAP_SERVER_IMPORT __declspec(dllimport)
  #endif
  #ifdef PCD_MAP_SERVER_BUILDING_DLL
    #define PCD_MAP_SERVER_PUBLIC PCD_MAP_SERVER_EXPORT
  #else
    #define PCD_MAP_SERVER_PUBLIC PCD_MAP_SERVER_IMPORT
  #endif
  #define PCD_MAP_SERVER_PUBLIC_TYPE PCD_MAP_SERVER_PUBLIC
  #define PCD_MAP_SERVER_LOCAL
#else
  #define PCD_MAP_SERVER_EXPORT __attribute__ ((visibility("default")))
  #define PCD_MAP_SERVER_IMPORT
  #if __GNUC__ >= 4
    #define PCD_MAP_SERVER_PUBLIC __attribute__ ((visibility("default")))
    #define PCD_MAP_SERVER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define PCD_MAP_SERVER_PUBLIC
    #define PCD_MAP_SERVER_LOCAL
  #endif
  #define PCD_MAP_SERVER_PUBLIC_TYPE
#endif

#endif  // PCD_MAP_SERVER__VISIBILITY_CONTROL_HPP_
