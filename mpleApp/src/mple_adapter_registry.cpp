#include <unordered_map>
#include "mple_adapter_registry.hpp"

namespace mple {

/// \brief Global map to store Adapter*'s keyed by string name
static std::unordered_map<std::string, Adapter*> g_adapter_registry;

void register_adapter(Adapter* adapter, const std::string& name) {
    printf("Registering adapter: %s\n", name.c_str());
    g_adapter_registry[name] = adapter;
}

Adapter* find_adapter(const std::string& name) {
    auto it = g_adapter_registry.find(name);
    if (it != g_adapter_registry.end()) {
        return it->second;
    }
    return nullptr;
}

} // namespace mple
