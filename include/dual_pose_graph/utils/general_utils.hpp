#ifndef DUAL_POSE_GRAPH__UTILS__GENERAL_UTILS_HPP_
#define DUAL_POSE_GRAPH__UTILS__GENERAL_UTILS_HPP_

#include <iostream>

#define DPG_RESET_COLOR "\033[0m"
#define DPG_RED_COLOR "\033[0;31m"
#define DPG_GREEN_COLOR "\033[1;32m"
#define DPG_YELLOW_COLOR "\033[1;33m"
#define DPG_CYAN_COLOR "\033[0;36m"

#define ERROR(x) \
  std::cerr << DPG_RED_COLOR << "[ERROR] " << x << std::endl \
            << "\t  at line " << __LINE__ << " in function " << __func__ << DPG_RESET_COLOR \
            << std::endl

#define WARN(x) std::cout << DPG_YELLOW_COLOR << "[WARN] " << x << DPG_RESET_COLOR << std::endl

#define FLAG(x)
#define DEBUG(x)
#define INFO(x)
#define PARAM(x) std::cout << DPG_CYAN_COLOR << "[PARAMS] " << x << DPG_RESET_COLOR << std::endl

#define NAME(x) "[" << x << "] "

#define FLAG_GRAPH(x) FLAG(NAME(name_) << x)
#define WARN_GRAPH(x) WARN(NAME(name_) << x)
#define INFO_GRAPH(x) INFO(NAME(name_) << x)
#define DEBUG_GRAPH(x) DEBUG(NAME(name_) << x)
#define ERROR_GRAPH(x) ERROR(NAME(name_) << x)

#define PRINT_VAR(var) #var << " = " << var

#endif  // DUAL_POSE_GRAPH__UTILS__GENERAL_UTILS_HPP_
