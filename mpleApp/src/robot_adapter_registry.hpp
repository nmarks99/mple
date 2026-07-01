#include "robot_adapter.hpp"
#include <string>

void register_adapter(RobotAdapter* adapter, const std::string& name);
RobotAdapter* find_adapter(const std::string& name);
