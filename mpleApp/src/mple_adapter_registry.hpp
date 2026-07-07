#include "mple_adapter.hpp"
#include <string>

void register_adapter(MpleAdapter* adapter, const std::string& name);
MpleAdapter* find_adapter(const std::string& name);
