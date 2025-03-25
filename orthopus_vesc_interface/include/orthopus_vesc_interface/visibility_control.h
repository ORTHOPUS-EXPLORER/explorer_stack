#ifndef ORTHOPUS_ROS__VISIBILITY_CONTROL_H_
#define ORTHOPUS_ROS__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ORTHOPUS_ROS_EXPORT __attribute__ ((dllexport))
    #define ORTHOPUS_ROS_IMPORT __attribute__ ((dllimport))
  #else
    #define ORTHOPUS_ROS_EXPORT __declspec(dllexport)
    #define ORTHOPUS_ROS_IMPORT __declspec(dllimport)
  #endif
  #ifdef ORTHOPUS_ROS_BUILDING_LIBRARY
    #define ORTHOPUS_ROS_PUBLIC ORTHOPUS_ROS_EXPORT
  #else
    #define ORTHOPUS_ROS_PUBLIC ORTHOPUS_ROS_IMPORT
  #endif
  #define ORTHOPUS_ROS_PUBLIC_TYPE ORTHOPUS_ROS_PUBLIC
  #define ORTHOPUS_ROS_LOCAL
#else
  #define ORTHOPUS_ROS_EXPORT __attribute__ ((visibility("default")))
  #define ORTHOPUS_ROS_IMPORT
  #if __GNUC__ >= 4
    #define ORTHOPUS_ROS_PUBLIC __attribute__ ((visibility("default")))
    #define ORTHOPUS_ROS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ORTHOPUS_ROS_PUBLIC
    #define ORTHOPUS_ROS_LOCAL
  #endif
  #define ORTHOPUS_ROS_PUBLIC_TYPE
#endif

#endif  // ORTHOPUS_ROS__VISIBILITY_CONTROL_H_
