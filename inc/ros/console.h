#ifndef ROS_CONSOLE_STUB_H
#define ROS_CONSOLE_STUB_H

#include <cassert>
#include <cstdio>

#define ROS_ASSERT_MSG(cond, ...) assert(cond)
#define ROS_DEPRECATED __attribute__((deprecated))

#define ROS_WARN_ONCE(...)                                      \
  do {                                                          \
    static bool _ros_warn_once_shown = false;                   \
    if (!_ros_warn_once_shown) {                                \
      fprintf(stderr, "[WARN] " __VA_ARGS__);                   \
      fprintf(stderr, "\n");                                    \
      _ros_warn_once_shown = true;                              \
    }                                                           \
  } while (0)

#endif
