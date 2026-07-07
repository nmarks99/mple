#include "mple_adapter_registry.hpp"
#include <unordered_map>

static std::unordered_map<std::string, MpleAdapter*> g_adapter_registry;

void register_adapter(MpleAdapter* adapter, const std::string& name) {
    printf("Registering adapter: %s\n", name.c_str());
    g_adapter_registry[name] = adapter;
}

MpleAdapter* find_adapter(const std::string& name) {
    auto it = g_adapter_registry.find(name);
    if (it != g_adapter_registry.end()) {
        return it->second;
    }
    return nullptr;
}
