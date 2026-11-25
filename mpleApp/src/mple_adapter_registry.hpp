#pragma once
#include <string>
#include "mple_adapter.hpp"

namespace mple {

/// \brief Registers an Adapter with the given name
void register_adapter(Adapter* adapter, const std::string& name);

/// \brief Finds an Adapter by name
/// \return Pointer to a registered Adapter if found, else nullptr
Adapter* find_adapter(const std::string& name);

} // namespace mple
