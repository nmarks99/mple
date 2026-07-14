#include <string>
#include "mple_adapter.hpp"

/// \brief Registers a MpleAdapter with the given name
void register_adapter(MpleAdapter* adapter, const std::string& name);

/// \brief Finds a MpleAdapter by name
/// \return Pointer to a registered MpleAdapter if found, else nullptr
MpleAdapter* find_adapter(const std::string& name);
